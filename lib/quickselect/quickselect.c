#include <solaris_ina228.h>

static float scratch[SOLARIS_RING_BUFFER_SIZE];

// swap two values
static inline void swap_float(float *a, float *b)
{
    float t = *a;
    *a = *b;
    *b = t;
}

// quickselect algorithm. finds the kth largest item in the given float array
// This will be used to find the median of our ring buffer in the energy monitor unit code
static float quickselect(float *a, int n, int k)
{
    int l = 0;
    int m = n - 1;

    while (l < m)
    {
        float x = a[k]; /* pivot */
        int i = l;
        int j = m;
        do
        {
            while (a[i] < x)
                i++;
            while (x < a[j])
                j--;
            if (i <= j)
            {
                swap_float(&a[i], &a[j]);
                i++;
                j--;
            }
        } while (i <= j);

        if (j < k)
            l = i;
        if (k < i)
            m = j;
    }
    return a[k];
}

float median_inplace(float *a, int n)
{

    if (n & 1)
    {
        /* odd: the single middle element */
        return quickselect(a, n, n / 2);
    }

    /* even: average the two middle elements.
     * quickselect(k = n/2) leaves the n/2 smallest values in a[0 .. n/2 - 1],
     * so the lower-middle element is simply the max of that partition -- no
     * need for a second full quickselect. */
    float hi = quickselect(a, n, n / 2);
    float lo = a[0];
    for (int i = 1; i < n / 2; i++)
    {
        if (a[i] > lo)
            lo = a[i];
    }
    return 0.5f * (lo + hi);
}

// copy the elements you are comparing against in your buffer and store it in the "scratch" buffer
float median_copy(const float *src, int n)
{
    for (int i = 0; i < n; i++)
    {
        scratch[i] = src[i];
    }
    return median_inplace(scratch, n);
}

// copy elements from the buffer based off size and the ring buffer and copy elements you need into scratch buffer and find the median
float solaris_windowed_median(const float *ring, int ring_size,
                              int head_idx, int window)
{

    for (int i = 0; i < window; i++)
    {
        int idx = head_idx - 1 - i;
        idx %= ring_size;
        if (idx < 0)
            idx += ring_size; /* C's % can be negative */
        scratch[i] = ring[idx];
    }
    return median_inplace(scratch, window);
}
