/*
 * Telemetry notification payload layout (per page, little-endian):
 *   [0]      page_index    (uint8_t)
 *   [1]      total_pages   (uint8_t)
 *   [2]      record_count  (uint8_t)  records in this page
 *   [3..]    records       (9 bytes each):
 *              [0..3]  timestamp         (uint32_t)
 *              [4]     cpu_temp          (uint8_t)
 *              [5]     battery_percent   (uint8_t)
 *              [6]     distance_m        (uint8_t)
 *              [7]     net_power_gain_w  (int8_t)
 *
 * Control characteristic write commands (1 byte):
 *   0x01 = REQUEST_PAGE  — start transfer or request first page
 *   0x02 = ACK_PAGE      — page received OK, send next
 *   0x03 = CLEAR_LOG     — all pages received and saved, wipe flash
 *
 * Time sync characteristic write payload:
 *   [0..3] unix timestamp (uint32_t, little-endian)
 *
 * Weather forecast characteristic write payload (152 bytes, little-endian):
 *   [0..3]   sunrise  (uint32_t)
 *   [4..7]   sunset   (uint32_t)
 *   [8..151] 24 x { time(uint32_t) cloud_cover_pct(uint8) precip_probability_pct(uint8) }
 */

#include "gatt_svc.h"

#include "solaris_common.h"
#include <solaris_mode.h>
#include "solaris_telemetry.h"
#include "solaris_weather.h"
#include "solaris_manual_ctrl.h"
#include <motor_driver.h>
#include "shared_resources.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include <sys/time.h>
#include "os/os_mbuf.h"

#define TAG "GATT_SVC"

/* ── Control command bytes ─────────────────────────────────────────────── */
#define CMD_REQUEST_PAGE 0x01
#define CMD_ACK_PAGE 0x02
#define CMD_CLEAR_LOG 0x03

/* Retry timeout: 5 seconds (FreeRTOS ticks) */
#define PAGE_RETRY_TIMEOUT_TICKS pdMS_TO_TICKS(5000)
#define PAGE_MAX_RETRIES 3

/* ── Telemetry page payload ────────────────────────────────────────────── */
#define TELEMETRY_RECORD_SIZE 8 /* bytes per encoded record */
#define TELEMETRY_PAGE_HEADER 3 /* page_index + total_pages + record_count */
#define TELEMETRY_PAGE_BUF_LEN \
    (TELEMETRY_PAGE_HEADER + SOLARIS_TELEMETRY_RECORDS_PER_PAGE * TELEMETRY_RECORD_SIZE)

/* ── UUIDs ─────────────────────────────────────────────────────────────── */
static const ble_uuid128_t solaris_svc_uuid =
    BLE_UUID128_INIT(0xd0, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

static const ble_uuid128_t solaris_telemetry_chr_uuid =
    BLE_UUID128_INIT(0xd1, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

static const ble_uuid128_t solaris_control_chr_uuid =
    BLE_UUID128_INIT(0xd2, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

static const ble_uuid128_t solaris_mode_chr_uuid =
    BLE_UUID128_INIT(0xd3, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

static const ble_uuid128_t solaris_time_sync_chr_uuid =
    BLE_UUID128_INIT(0xd4, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

static const ble_uuid128_t solaris_weather_chr_uuid =
    BLE_UUID128_INIT(0xd5, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

static const ble_uuid128_t solaris_manual_ctrl_chr_uuid =
    BLE_UUID128_INIT(0xd6, 0x00, 0x00, 0x00, 0xa1, 0xb2, 0xc3, 0xd4,
                     0xe5, 0xf6, 0x07, 0x18, 0x29, 0x3a, 0x4b, 0x5c);

/* ── Characteristic value handles ─────────────────────────────────────── */
static uint16_t telemetry_chr_val_handle;
static uint16_t control_chr_val_handle;
static uint16_t mode_chr_val_handle;
static uint16_t time_sync_chr_val_handle;
static uint16_t weather_chr_val_handle;
static uint16_t manual_ctrl_chr_val_handle;

/* ── Pagination state ──────────────────────────────────────────────────── */
static uint16_t telemetry_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool telemetry_notify_enabled = false;
static int current_page = 0;
static int retry_count = 0;
static TimerHandle_t retry_timer = NULL;
static struct ble_npl_event retry_event;

/* ── Forward declarations ──────────────────────────────────────────────── */
static int solaris_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);
static void send_current_page(void);
static void retry_timer_cb(TimerHandle_t xTimer);

/* ── GATT service table ────────────────────────────────────────────────── */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &solaris_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Telemetry: ESP → Phone via notifications */
                .uuid = &solaris_telemetry_chr_uuid.u,
                .access_cb = solaris_chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &telemetry_chr_val_handle,
            },
            {
                /* Control: Phone → ESP (REQUEST_PAGE / ACK_PAGE / CLEAR_LOG) */
                .uuid = &solaris_control_chr_uuid.u,
                .access_cb = solaris_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &control_chr_val_handle,
            },
            {
                /* Mode: read current mode or write to change it */
                .uuid = &solaris_mode_chr_uuid.u,
                .access_cb = solaris_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .val_handle = &mode_chr_val_handle,
            },
            {
                /* Time Sync: Phone → ESP, 4-byte unix timestamp */
                .uuid = &solaris_time_sync_chr_uuid.u,
                .access_cb = solaris_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &time_sync_chr_val_handle,
            },
            {
                /* Weather Forecast: Phone → ESP, 152-byte payload */
                .uuid = &solaris_weather_chr_uuid.u,
                .access_cb = solaris_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &weather_chr_val_handle,
            },
            {
                /* Manual Control: Phone → ESP, 2 bytes: throttle (int8) + steering (int8) */
                .uuid = &solaris_manual_ctrl_chr_uuid.u,
                .access_cb = solaris_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &manual_ctrl_chr_val_handle,
            },
            {0},
        },
    },
    {0},
};

/* ── Encoding helpers ──────────────────────────────────────────────────── */
static inline void put_u32_le(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v);
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v >> 16);
    dst[3] = (uint8_t)(v >> 24);
}

// get 32 bit value
static inline uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ── Pagination ────────────────────────────────────────────────────────── */
static void send_current_page(void)
{
    if (!telemetry_notify_enabled ||
        telemetry_conn_handle == BLE_HS_CONN_HANDLE_NONE)
    {
        return;
    }

    solaris_telemetry_t records[SOLARIS_TELEMETRY_RECORDS_PER_PAGE];
    int count = 0;
    int total_pages = solaris_telemetry_log_page_count();

    if (total_pages == 0)
    {
        /* Nothing to send — send a single empty page to signal done */
        uint8_t buf[TELEMETRY_PAGE_HEADER] = {0, 0, 0};
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
        if (om)
            ble_gatts_notify_custom(telemetry_conn_handle,
                                    telemetry_chr_val_handle, om);
        return;
    }

    if (!solaris_telemetry_log_get_page(current_page, records, &count))
    {
        ESP_LOGE(TAG, "failed to get telemetry page %d", current_page);
        return;
    }

    /* Flat packet layout:
     *   buf[0] = current_page
     *   buf[1] = total_pages
     *   buf[2] = record count
     *   buf[3..N] = records, each TELEMETRY_RECORD_SIZE bytes:
     *     +0..3  timestamp        (uint32_t, little-endian)
     *     +4     cpu_temp         (uint8_t)
     *     +5     battery_percent  (uint8_t)
     *     +6     distance_m       (uint8_t)
     *     +7     net_power_gain_w (int8_t)
     */
    uint8_t buf[TELEMETRY_PAGE_BUF_LEN];
    buf[0] = (uint8_t)current_page;
    buf[1] = (uint8_t)total_pages;
    buf[2] = (uint8_t)count;

    /* p is a write cursor starting just after the 3-byte header */
    uint8_t *p = &buf[TELEMETRY_PAGE_HEADER];
    for (int i = 0; i < count; i++)
    {
        put_u32_le(p, records[i].timestamp);
        p[4] = records[i].cpu_temp;
        p[5] = records[i].battery_percent;
        p[6] = records[i].distance_m;
        p[7] = (uint8_t)records[i].net_power_gain_w;
        p += TELEMETRY_RECORD_SIZE; /* advance to next record slot */
    }

    uint16_t payload_len = TELEMETRY_PAGE_HEADER + (uint16_t)(count * TELEMETRY_RECORD_SIZE);
    // copy buf into an os_mbuf chain that NimBLE can use
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, payload_len);
    if (!om)
    {
        ESP_LOGE(TAG, "failed to allocate mbuf for telemetry page %d", current_page);
        return;
    }

    int rc = ble_gatts_notify_custom(telemetry_conn_handle,
                                     telemetry_chr_val_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "telemetry notify failed, rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "sent telemetry page %d/%d (%d records)",
             current_page + 1, total_pages, count);

    /* Start/reset the retry timer */
    if (retry_timer)
    {
        xTimerReset(retry_timer, 0);
    }
}

static void retry_event_cb(struct ble_npl_event *ev)
{
    (void)ev;
    send_current_page();
}

static void retry_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (retry_count >= PAGE_MAX_RETRIES)
    {
        ESP_LOGW(TAG, "page %d max retries reached, giving up", current_page);
        retry_count = 0;
        xTimerStop(retry_timer, 0);
        return;
    }

    ESP_LOGW(TAG, "no ACK for page %d, retrying (%d/%d)",
             current_page, retry_count + 1, PAGE_MAX_RETRIES);
    retry_count++;

    /* Dispatch to NimBLE host task — timer task stack is too small for NVS + notify */
    ble_npl_event_init(&retry_event, retry_event_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &retry_event);
}

/* ── Characteristic access callback ───────────────────────────────────── */
static int handle_control_write(struct ble_gatt_access_ctxt *ctxt)
{
    if (ctxt->om->om_len < 1)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t cmd = ctxt->om->om_data[0];

    switch (cmd)
    {
    case CMD_REQUEST_PAGE:
        ESP_LOGI(TAG, "REQUEST_PAGE received, starting transfer");
        solaris_telemetry_load();
        current_page = 0;
        retry_count = 0;
        send_current_page();
        break;

    case CMD_ACK_PAGE:
        ESP_LOGI(TAG, "ACK for page %d", current_page);
        if (retry_timer)
            xTimerStop(retry_timer, 0);
        retry_count = 0;
        current_page++;

        if (current_page >= solaris_telemetry_log_page_count())
        {
            ESP_LOGI(TAG, "all pages sent, waiting for CLEAR_LOG");
        }
        else
        {
            send_current_page();
        }
        break;

    case CMD_CLEAR_LOG:
        ESP_LOGI(TAG, "CLEAR_LOG received, wiping telemetry log");
        if (retry_timer)
            xTimerStop(retry_timer, 0);
        solaris_telemetry_log_clear();
        current_page = 0;
        retry_count = 0;
        break;

    default:
        ESP_LOGW(TAG, "unknown control command: 0x%02x", cmd);
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int handle_mode_read(struct ble_gatt_access_ctxt *ctxt)
{
    uint8_t mode = (uint8_t)solaris_mode_get();
    int rc = os_mbuf_append(ctxt->om, &mode, sizeof(mode));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int handle_mode_write(struct ble_gatt_access_ctxt *ctxt)
{
    if (ctxt->om->om_len != 1)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t requested_mode = ctxt->om->om_data[0];
    if (!solaris_mode_set_from_u8(requested_mode))
        return BLE_ATT_ERR_UNLIKELY;

    solaris_event_t evt = {.type = SOLARIS_EVENT_MODE_CHANGE, .mode = (uint8_t)solaris_mode_get()};
    if (xQueueSend(xEventQueue, &evt, (TickType_t)(10)) != pdPASS)
    {
        ESP_LOGE(TAG, "Queue was unable to send");
    }

    ESP_LOGI(TAG, "mode changed to %s", solaris_mode_to_str(solaris_mode_get()));
    return 0;
}

// recieve 32 bit unix timestamp from phone and set the proper time of day
static int handle_time_sync_write(struct ble_gatt_access_ctxt *ctxt)
{
    if (ctxt->om->om_len != 4)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint32_t unix_ts = get_u32_le(ctxt->om->om_data);
    ESP_LOGI(TAG, "time sync received: %lu", (unsigned long)unix_ts);
    struct timeval tv = {.tv_sec = (time_t)unix_ts, .tv_usec = 0};
    // Now we can get timestamp at any point by including time.h and using (uint32_t)time(NULL)
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "system clock set to %lu", (unsigned long)unix_ts);
    return 0;
}

static int handle_weather_write(struct ble_gatt_access_ctxt *ctxt)
{
    if (ctxt->om->om_len != SOLARIS_WEATHER_PAYLOAD_LEN)
    {
        ESP_LOGE(TAG, "weather payload wrong length: %d (expected %d)",
                 ctxt->om->om_len, SOLARIS_WEATHER_PAYLOAD_LEN);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (!solaris_weather_set_from_ble(ctxt->om->om_data, ctxt->om->om_len))
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    solaris_weather_t g_weather;
    solaris_weather_get(&g_weather);
    ESP_LOGI(TAG, "weather forecast received — sunrise=%u sunset=%u",
             g_weather.sunrise, g_weather.sunset);
    for (int i = 0; i < SOLARIS_WEATHER_FORECAST_HOURS; i++)
    {
        ESP_LOGI(TAG, "  [%02d] t=%u cloud=%u%% precip=%u%%",
                 i,
                 g_weather.forecast[i].time,
                 g_weather.forecast[i].cloud_cover_pct,
                 g_weather.forecast[i].precip_probability_pct);
    }

    return 0;
}

// Tracks whether *this* handler currently owns actuator_mutex on behalf of
// the drive motors. GATT write callbacks all run on the NimBLE host task, so
// it's safe for the mutex to be taken on one call and given back on a later
// one — ownership is per-task, not per-invocation.
static bool s_drive_mutex_held = false;

static int handle_manual_ctrl_write(struct ble_gatt_access_ctxt *ctxt)
{
    if (ctxt->om->om_len != 2)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    solaris_manual_ctrl_set((int8_t)ctxt->om->om_data[0],
                            (int8_t)ctxt->om->om_data[1]);
    int8_t throttle;
    int8_t steering;
    solaris_manual_ctrl_get(&throttle, &steering);

    // Assume only throttle or steering is used. Not both at the same time

    // If both throttle and steering commands are 0, stop moving all drive motors
    if (throttle == 0 && steering == 0)
    {
        if (s_drive_mutex_held)
        {
            stop_motor(MOTOR_LEFT_ID);
            stop_motor(MOTOR_RIGHT_ID);
            xSemaphoreGive(actuator_mutex);
            s_drive_mutex_held = false;
        }
        ESP_LOGI(TAG, "throttle: %d\tsteering:%d", throttle, steering);
        return 0;
    }

    // Non-stop command: acquire the mutex once, on the transition into
    // motion. While already driving we just keep updating speed/direction
    // on the lock we already hold.
    if (!s_drive_mutex_held)
    {
        if (xSemaphoreTake(actuator_mutex, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "actuator busy (panel moving), ignoring drive command");
            return 0;
        }
        s_drive_mutex_held = true;
    }

    if (throttle > 0 && s_drive_mutex_held)
    { // move forward
        motor_go_forward(MOTOR_LEFT_ID, ((float)throttle / 127.0) * .75);
        motor_go_backward(MOTOR_RIGHT_ID, ((float)throttle / 127.0) * .75);
    }
    else if (throttle < 0 && s_drive_mutex_held)
    { // move backward
        motor_go_backward(MOTOR_LEFT_ID, ((float)abs(throttle) / 127.0) * .75);
        motor_go_forward(MOTOR_RIGHT_ID, ((float)abs(throttle) / 127.0) * .75);
    }
    else if (steering > 0 && s_drive_mutex_held)
    { // turn right
        motor_go_forward(MOTOR_LEFT_ID, ((float)steering / 127.0) * .75);
        motor_go_forward(MOTOR_RIGHT_ID, ((float)steering / 127.0) * .75);
    }
    else if (steering < 0 && s_drive_mutex_held)
    { // turn left
        motor_go_backward(MOTOR_LEFT_ID, ((float)abs(steering) / 127.0) * .75);
        motor_go_backward(MOTOR_RIGHT_ID, ((float)abs(steering) / 127.0) * .75);
    }

    ESP_LOGI(TAG, "throttle: %d\tsteering:%d", throttle, steering);

    return 0;
}

static int solaris_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)arg;

    if (attr_handle == control_chr_val_handle)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
            return handle_control_write(ctxt);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    if (attr_handle == mode_chr_val_handle)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
            return handle_mode_read(ctxt);
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
            return handle_mode_write(ctxt);
    }
    if (attr_handle == time_sync_chr_val_handle)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
            return handle_time_sync_write(ctxt);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    if (attr_handle == weather_chr_val_handle)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
            return handle_weather_write(ctxt);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    if (attr_handle == manual_ctrl_chr_val_handle)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
            return handle_manual_ctrl_write(ctxt);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ── Public functions ──────────────────────────────────────────────────── */
void solaris_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];
    (void)arg;

    switch (ctxt->op)
    {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "registering characteristic %s with def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
        break;
    default:
        assert(0);
        break;
    }
}

void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    if (event->subscribe.attr_handle == telemetry_chr_val_handle)
    {
        telemetry_conn_handle = event->subscribe.conn_handle;
        telemetry_notify_enabled = (event->subscribe.cur_notify ||
                                    event->subscribe.cur_indicate);
        ESP_LOGI(TAG, "telemetry notifications %s",
                 telemetry_notify_enabled ? "enabled" : "disabled");
    }
}

void gatt_svc_on_disconnect(void)
{
    if (retry_timer)
        xTimerStop(retry_timer, 0);
    telemetry_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    telemetry_notify_enabled = false;
    current_page = 0;
    retry_count = 0;
}

int gatt_svc_init(void)
{
    int rc;

    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
        return rc;

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
        return rc;

    retry_timer = xTimerCreate("ble_retry", PAGE_RETRY_TIMEOUT_TICKS,
                               pdTRUE, NULL, retry_timer_cb);
    if (!retry_timer)
    {
        ESP_LOGE(TAG, "failed to create retry timer");
        return -1;
    }

    return 0;
}
