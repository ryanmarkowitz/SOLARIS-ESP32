#pragma once

float median_inplace(float *a, int n);
float median_copy(const float *src, int n);
float solaris_windowed_median(const float *ring, int ring_size,
                              int head_idx, int window);
float median_copy(const float *src, int n);