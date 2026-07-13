# SOLARIS — Firmware Overview

SOLARIS is an ESP32-S3 firmware project (PlatformIO + ESP-IDF/`espidf` framework) for a
solar-tracking robot: a mobile chassis (4-wheel tank drive) carrying a solar panel on a
2-axis pan/tilt gimbal. The firmware chases sunlight with the panel, exposes a BLE control
and telemetry interface for a phone app, and is architecturally scaffolded (but not yet
finished) to relocate the whole robot to avoid shade and maximize power output.

This document is meant to give a complete, ground-truth picture of the codebase — including
what's actually wired up vs. what's stubbed, dead, or buggy — so that anyone (human or AI)
picking up this project or writing a report about it has full context without having to
re-read every file.

Target board: `esp32-s3-devkitc-1` (see `platformio.ini`). Build flag: `-I include`.

---

## 1. Repository Layout

```
SOLARIS-ESP32/
├── platformio.ini            PlatformIO project config (espidf framework, esp32-s3-devkitc-1)
├── CMakeLists.txt             Top-level ESP-IDF project wrapper
├── src/
│   ├── main.c                 app_main() — entry point, task creation
│   └── CMakeLists.txt         auto-generated component registration
├── include/
│   ├── shared_resources.h     cross-task globals (mutex, queue, task handles)
│   ├── solaris_common.h       shared include bundle + DEVICE_NAME
│   └── README                 stock PlatformIO boilerplate (unedited)
└── lib/
    ├── driver/                top-level mode state-machine task
    ├── encoders/               PCNT-based quadrature/pulse position tracking
    ├── gap/                    BLE GAP (advertising/connection)
    ├── gatt_svc/               BLE GATT service — the phone↔ESP32 protocol
    ├── log_telemetry/          NVS telemetry snapshot logger (currently DEAD CODE, not started)
    ├── motor_driver/           MCPWM 6-motor driver (pan, tilt, FL/FR/RL/RR wheels)
    ├── nimble_init/            NimBLE stack bring-up
    ├── solaris_icm20948/       IMU driver (ICM-20948 + AK09916 mag) — implemented, not yet
    │                           wired in; likely heading feedback for AUTOMATIC mode's
    │                           "rotate to face direction" step (§3, `lib/driver/`)
    ├── solaris_ina228/         Power/current/SOC monitor (INA228) — implemented, not yet
    │                           wired in; this IS the power sensor AUTOMATIC mode's
    │                           shade-detection algorithm is built around (§3, `lib/driver/`)
    ├── solaris_manual_ctrl/    Manual throttle/steering global state
    ├── solaris_mock_data/      Fake historical telemetry generator (dev tool, never called)
    ├── solaris_mode/           Operating-mode enum + state holder
    ├── solaris_telemetry/      In-RAM telemetry log + BLE pagination backend
    ├── solaris_ultrasonic/     4x MB1020 analog ultrasonic ranging — implemented, not yet
    │                           wired in; not referenced in the AUTOMATIC-mode design comment,
    │                           but a plausible fit for obstacle sensing while driving
    ├── solaris_weather/        Weather forecast storage (BLE-fed) — cloud-cover % is an input
    │                           to AUTOMATIC mode's shade check (§3), not yet consumed
    └── solar_tracking/         Phototransistor-driven pan/tilt sun-tracking task; also holds
                                the chassis-direction sensors AUTOMATIC mode uses for its
                                "which direction to move" step
```

There is no `library.json`/`CMakeLists.txt` inside any `lib/*` folder — PlatformIO's Library
Dependency Finder (LDF) auto-discovers each library from `#include` statements in `src/main.c`
and cross-includes between libraries.

---

## 2. High-Level Architecture

### 2.1 Boot sequence (`src/main.c :: app_main()`)

```c
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second delay
    nimble_init();
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); err = nvs_flash_init(); }
    ESP_ERROR_CHECK(err);
    encoder_init();
    motor_init();
    bool pt_ok = (solaris_pt_init(&pt_cfg, &pt) == ESP_OK);
    if (!pt_ok) ESP_LOGE(TAG, "Phototransistor init failed - skipping that subsystem");
    actuator_mutex = xSemaphoreCreateMutex();
    xModeQueue = xQueueCreate(MODE_QUEUE_LENGTH, sizeof(solaris_mode_t));
    xTaskCreate(solar_tracking, "solar tracking", 4096, NULL, 13, &xSolarTracking);
    xTaskCreate(driver_function, "driving function", 4096, NULL, 10, &xDriverFunction);
    // xTaskCreate(test_motor, "test drive", 4096, NULL, 8, NULL);
}
```

Note at `main.c:61`: `// IF YOU GET STACK OVERFLOW ERRORS CHANGE 4096 TO HIGHER NUMBER AS THIS IS
THE STACK DEPTH ALLOCATION` — an operational note worth keeping in mind if tasks start
crashing after adding more logic to them.

`main.c` also `#include`s headers for the IMU (`solaris_icm20948.h`), INA228
(`solaris_ina228.h`), and ultrasonic (`solaris_ultrasonic.h`) modules — **but never calls
their init/read functions.** These are currently dead includes (see §5).

### 2.2 FreeRTOS tasks that actually run

| Task | Priority | Created in | Purpose |
|---|---|---|---|
| `solar_tracking` | 13 | `main.c` | Event-driven pan/tilt sun-chasing (waits on task notification) |
| `driver_function` | 10 | `main.c` | Mode state machine (STATIONARY / AUTOMATIC / MANUAL) |
| NimBLE Host | 5 (core 0) | `nimble_init()` | Runs `nimble_port_run()` — the BLE stack |
| `encoder_handler` | 18 | `encoder_init()` | Services PCNT limit/overflow events from ISR via queue |

**Tasks written but never started (dead in this checkout):**
- `test_motor` (`lib/motor_driver/motor_driver.c`) — creation commented out in `main.c:64`.
- `log_telemetry` (`lib/log_telemetry/log_telemetry.c`) — fully written, no `xTaskCreate` call
  anywhere in the repo. Confirmed by commit `182c395`: *"added the log telemetry task. It is
  not completed yet, but the nvs outline is completed."*

### 2.3 Shared state (`include/shared_resources.h`)

```c
extern SemaphoreHandle_t actuator_mutex; // Only drive motors or panel motors are allowed to move, don't let them move at the same time
extern QueueHandle_t xModeQueue;
extern TaskHandle_t xSolarTracking;
extern TaskHandle_t xDriverFunction;
```

- **`actuator_mutex`** — mutual exclusion between the 4 drive motors (taken in
  `gatt_svc.c`'s manual-control handler) and the pan/tilt panel motors (taken in
  `solar_tracking.c`). Both sites use a **non-blocking** `xSemaphoreTake(actuator_mutex, 0)`,
  so if the mutex is held elsewhere, the caller just logs and skips the move rather than
  waiting — manual drive commands can be silently dropped if the panel is mid-track, and
  vice versa.
- **`xModeQueue`** — queue of `solaris_mode_t` (length 10), fed by the BLE "mode"
  characteristic write handler, drained by `driver_function`.
- **`xSolarTracking` / `xDriverFunction`** — task handles used for
  `xTaskNotifyGive`/`xTaskNotifyWait` signaling (the driver task pokes the tracking task when
  entering STATIONARY mode).

`include/solaris_common.h` bundles common NimBLE/NVS/FreeRTOS includes and defines
`DEVICE_NAME "SOLARIS"`. `include/README` and `lib/README` are unedited PlatformIO
boilerplate — no project content.

### 2.4 Data flow summary

1. A phone app connects over BLE. `gap.c` handles advertising/connection; `gatt_svc.c`
   exposes 6 characteristics (telemetry notify, control, mode, time-sync, weather, manual
   control).
2. **Mode writes** go into `xModeQueue` → consumed by `driver_function`:
   - `SOLARIS_MODE_STATIONARY` → pokes `solar_tracking` to run one pan/tilt chase pass.
   - `SOLARIS_MODE_AUTOMATIC` → **unimplemented stub.** The design (fully laid out in §3,
     `lib/driver/`) is a shade-avoidance loop: detect a sustained power drop via INA228,
     confirm against cloud-cover %, pick a direction from the chassis phototransistors,
     rotate and drive in up to 3 steps, and check after each step whether the power gain
     recovered — stopping either on recovery or after 3 failed steps.
   - `SOLARIS_MODE_MANUAL` → no-op in `driver_function`; actual manual driving happens
     directly in the BLE write callback (`handle_manual_ctrl_write` in `gatt_svc.c`), bypassing
     the driver task entirely.
3. **Encoders** (PCNT-based) track pan/tilt/wheel positions; an ISR pushes limit/overflow
   events to `encoder_handler`, which stops panel motors at soft limits.
4. **Motor driver** uses MCPWM (3 operators, 6 motors) to drive dir+PWM H-bridge motor
   controllers for pan, tilt, and 4 wheels.
5. **Telemetry**: `solaris_telemetry` holds an in-RAM log (loaded from NVS) paged out over
   BLE with a retry/ACK protocol. `solaris_mock_data` can seed fake historical records into
   NVS for demos (never actually invoked in code — presumably run manually during dev).
6. **Weather** and **manual control** data are simple global structs written by BLE and read
   elsewhere — weather is stored but **not currently consumed** by any tracking/driving logic.
7. **IMU, power monitor (INA228), and ultrasonic** are complete, self-contained sensor-driver
   libraries (`_init/_read/_log/_deinit` APIs) — **none of them are wired into `app_main()`.**
   Only the phototransistor library (`solaris_pt_init`) is actually initialized at boot. INA228
   is the confirmed power sensor for the AUTOMATIC-mode plan (§3, `lib/driver/`); IMU is a
   plausible fit for rotation feedback; ultrasonic isn't mentioned in the plan at all.

---

## 3. Module-by-Module Breakdown

### `src/main.c` — Entry point
Delays 3s, brings up BLE, NVS, encoders, motors, phototransistor sensors; creates the
mutex/queue; spawns `solar_tracking` and `driver_function`. See §2.1 for full listing.

### `include/` — Shared headers
- `shared_resources.h` — cross-task globals (§2.3).
- `solaris_common.h` — shared includes + `DEVICE_NAME`.
- `README` — stock, unmodified.

### `lib/driver/` — Mode-driven top-level task
**State: stubbed (AUTOMATIC), functional passthrough (STATIONARY), intentional no-op (MANUAL).**

`driver_function()` loops forever, switching on a locally cached `solaris_mode`:

```c
case SOLARIS_MODE_AUTOMATIC:
    /*
    1. check if in shade
        a. if no shade detected move panel if necessary and skip to last step
        b. if shade is detected start new moving process step 2
    2. Determine which direction SOLARIS should move
    3. Rotate the robot to match that direction
    4. Move the robot in that direction for a period of time.
    5. Check if there was an expected power gain.
        a. If expected power gain is achieved skip stop moving robot
        b. If expected power gain is not achieved try moving forward again
    */
    break;
case SOLARIS_MODE_MANUAL:
    break; // handled entirely in gatt_svc.c's handle_manual_ctrl_write
case SOLARIS_MODE_STATIONARY:
    xTaskNotifyGive(xSolarTracking);
    break;
```

After the switch: `xQueueReceive(xModeQueue, &solaris_mode, pdMS_TO_TICKS(250))` then
`vTaskDelay(250ms)` — loop cadence up to ~500ms/iteration.

**The full AUTOMATIC-mode plan**, verbatim from the comment block at the top of `driver.c`
(lines 10-23) — this is the actual spec, not just the 5-step outline inside the switch-case:

> Framework for detecting if shade:
> First make sure we aren't in sunrise / sunset window
> Check energy monitor every second and store power it sees in a ring buffer
> Every 3 minutes we will check the median of the past 30 seconds of info vs last 3 minutes. if there is a significant drop move to next checks
> If cloud coverage percentage is low, move
> Move in direction phototransistors on chassis recommends
> Move in steps. we'll do 3 steps.
> At each let energy monitor take 30 samples After settling panel. Then check if power gain matches or comes close to what the drop was
> If we recieved power gain similar to the drop, stop movement
> If we didn't recieve power gains, do another step.
> If after 3 steps we don't recieve power gains, cut losses. Don't let sunk cost break the system.

Mapped onto the modules that already exist in the repo:

- **Shade detection** — sample `solaris_ina228` (power monitor) once/second into a ring
  buffer; every 3 minutes, compare the median of the last 30s against the median of the last
  3min (outside a sunrise/sunset window). A significant drop = shade detected.
- **Direction decision** — cross-check the drop against `solaris_weather`'s stored cloud-cover
  %, then take the direction hint from the chassis-mounted phototransistors (`solar_tracking`'s
  `solaris_pt_read(..., false)` path — sensor indices 4-7 / FL,FR,RL,RR — implemented but never
  invoked yet, see §3/§5).
- **Rotate + move** — turn the chassis to face the recommended direction (rotation feedback is
  the likely intended use of the currently-unwired `solaris_icm20948` IMU, though the design
  comment doesn't name it explicitly), then drive forward for one of 3 discrete steps using
  `motor_driver`'s wheel motors.
- **Verify** — after each step, let the INA228 settle and take 30 samples; if the power gain
  roughly matches the drop that triggered the move, stop (shade avoided). If not, take another
  step. After 3 steps with no recovery, stop anyway — "don't let sunk cost break the system."
- `solaris_ultrasonic` is implemented but **not referenced anywhere in this design comment** —
  it's not a confirmed part of the AUTOMATIC-mode spec, just a plausible candidate (e.g.
  obstacle sensing while driving) if the plan is extended later.

**None of this is implemented** — the switch-case body is empty aside from the 5-step outline
comment.

`driver.h` also declares `void write_log_and_buffers();` — **no implementation exists
anywhere in the codebase.** Likely the planned hook for the "store it in a ring buffer" /
telemetry-logging part of the plan above.

### `lib/encoders/` — Quadrature/pulse position tracking (PCNT)
**State: functional core, with two persistence no-ops and one design TODO.**

4 encoders: `PAN_ENCODER_ID`(0), `TILT_ENCODER_ID`(1), `FL_ENCODER_ID`(2), `FR_ENCODER_ID`(3).

```c
static const encoder_pins_t encoder_pins[NUM_ENCODERS] = {
    {.encoder_gpio = 37, .dir_gpio = 18},
    {.encoder_gpio = 35, .dir_gpio = 38},
    {.encoder_gpio = 42, .dir_gpio = -1},
    {.encoder_gpio = 45, .dir_gpio = -1}};
```
⚠️ The header comment above this table (`encoders.c:31-35`) says `Pan encoder - Pin 30 | Dir -
Pin 11`, `Tilt encoder - Pin 28 | Dir - Pin 31`, `FL encoder - Pin 35`, `FR encoder - Pin 26` —
**this does not match the actual array** (stale documentation; fix or remove).

- `HIGH_LIMIT`/`LOW_LIMIT` = ±32767/-32768 (int16 PCNT range); `HIGH_LIMIT_FR` = 28387
  (deliberately different "to avoid conflicting interrupts between FL encoder").
- Pan/tilt soft-limit watchpoints: `FORWARD_TARGET_PAN 2650` (360°), `BACKWARD_TARGET_PAN
  -2650`, `FORWARD_TARGET_TILT 350` (+30°), `BACKWARD_TARGET_TILT -350`.
- `on_encoder_limit_reached()` — PCNT ISR callback, pushes an `encoder_evt_t` to a queue from
  ISR context.
- `encoder_handler()` task (priority 18) — for PAN/TILT, calls `stop_motor()` +
  `panel_set_limit_state()` at soft limits; for FL/FR, increments overflow counters.
- Glitch filter set to 10µs (documented against max no-load speed: "30RPM with 5281 PPR at
  the Output Shaft... 378us between pulses").

**TODO** (`encoders.c:177`): `// TODO EVENTUALLY ALL ENCODERS NEED TO WORK WITH OPTOCOUPLER.
THE ELSE BLOCK WOULD BE THE RIGHT WAY TO DO IT`

**TODO / known-broken** (`encoders.c:348-351`, inside `get_distance_traveled()`):
```c
// TODO change the pulses to convert to m traveled
// This implementation will fail as is since there will be overflows.
// We need to convert pulses to distance traveled before summing the distance
// This is just temp code for idea of where to go next
```

**Position persistence is currently a no-op.** `load_position_from_flash()` reads
`pan_position`/`tilt_position` from NVS namespace `"panel"`, logs the value, but always
`return 0;` (line 301, with `// return position;` commented out just above) — the loaded
value is discarded. `save_position_to_flash()` similarly always writes `0`
(`int absolute_position = 0;` at line 318, with the real computation
`// int absolute_position = position + saved_pan_or_tilt;` commented out) — panel position is
**not actually remembered across power cycles**, even though all the NVS plumbing is built
out for it. This looks like an intentional safety stopgap left mid-refactor rather than a
finished feature — worth fixing before relying on position persistence.

### `lib/gap/` — BLE GAP (advertising & connection)
**State: fully functional, adapted from Espressif's NimBLE example.**

- `gap_init()` — `ble_svc_gap_init()` + sets device name to `DEVICE_NAME`.
- `adv_init()` — infers BT address, calls `start_advertising()`.
- `start_advertising()` — builds adv fields (flags, name, tx power auto, appearance
  `BLE_GAP_APPEARANCE_GENERIC_TAG`, LE peripheral role) and scan-response (address + an
  `esp_uri` still pointing at `https://espressif.com` — cosmetic leftover from the example
  this was adapted from, worth changing for a shipped product); advertises undirected
  connectable/discoverable at 500ms interval.
- `gap_event_handler()` — handles CONNECT, DISCONNECT (calls `gatt_svc_on_disconnect()` then
  restarts advertising), CONN_UPDATE, ADV_COMPLETE, NOTIFY_TX, SUBSCRIBE, MTU.

### `lib/gatt_svc/` — BLE GATT service (the primary protocol surface)
**State: the most mature/complete module in the repo (604 lines).**

Wire protocol, documented verbatim at the top of `gatt_svc.c:1-25`:

```
Telemetry notification payload layout (per page, little-endian):
  [0]      page_index    (uint8_t)
  [1]      total_pages   (uint8_t)
  [2]      record_count  (uint8_t)  records in this page
  [3..]    records       (11 bytes each): timestamp(u32) cpu_temp(u8)
             battery_percent(u8) distance_m(float32) net_power_gain_w(i8)
Control characteristic write commands (1 byte):
  0x01 = REQUEST_PAGE, 0x02 = ACK_PAGE, 0x03 = CLEAR_LOG
Time sync characteristic write payload: [0..3] unix timestamp
Weather forecast characteristic write payload (152 bytes, little-endian): ...
```
`distance_m` was widened from `uint8_t` to `float` (IEEE-754 binary32, little-endian) so
fractional meters aren't truncated; `TELEMETRY_RECORD_SIZE` is `11` and
`SOLARIS_TELEMETRY_RECORDS_PER_PAGE` was lowered to `20` to keep a full page under the
negotiated ATT MTU.

One primary service (UUID prefix `0xd0`) with 6 characteristics (128-bit UUIDs differing in
first byte):

| Char | UUID suffix | Properties | Purpose |
|---|---|---|---|
| Telemetry | `0xd1` | NOTIFY | Paginated historical log transfer |
| Control | `0xd2` | WRITE/WRITE_NO_RSP | Pagination protocol (REQUEST_PAGE/ACK_PAGE/CLEAR_LOG) |
| Mode | `0xd3` | READ+WRITE | Get/set `solaris_mode_t` |
| Time Sync | `0xd4` | WRITE | 4-byte unix timestamp → `settimeofday()` |
| Weather | `0xd5` | WRITE | 152-byte forecast blob |
| Manual Control | `0xd6` | WRITE_NO_RSP | 2 bytes: throttle int8, steering int8 |

**Pagination/retry:** `send_current_page()` builds a flat buffer and notifies via
`ble_gatts_notify_custom`; a 5s FreeRTOS software timer (`retry_timer`) resends up to
`PAGE_MAX_RETRIES` (3) if unacked. The retry is dispatched onto the NimBLE host's event queue
rather than run directly in the timer callback — explicit comment explains why
(`gatt_svc.c:286`): *"Dispatch to NimBLE host task — timer task stack is too small for NVS +
notify."*

**Manual drive mixing** (`handle_manual_ctrl_write`) — tank-style differential drive:

```c
if (throttle == 0 && steering == 0) { /* stop all 4 wheels */ }
else if (throttle > 0) { // forward: FL/RL forward, FR/RR backward, scaled to 75% max
    motor_go_forward(MOTOR_FL_ID, (throttle/127.0)*.75);
    motor_go_forward(MOTOR_RL_ID, (throttle/127.0)*.75);
    motor_go_backward(MOTOR_FR_ID, (throttle/127.0)*.75);
    motor_go_backward(MOTOR_RR_ID, (throttle/127.0)*.75);
}
else if (throttle < 0) { /* symmetric backward case */ }
else if (steering > 0) { /* turn right: all 4 wheels forward at steering magnitude */ }
else if (steering < 0) { /* turn left: all 4 wheels backward */ }
```
Explicit design assumption in comments: *"Assume only throttle or steering is used. Not both
at the same time."* Each branch takes `actuator_mutex` non-blocking before moving — drops the
command silently if the panel-tracking task currently holds it.

Other handlers: `handle_mode_write()` (validates + pushes to `xModeQueue`),
`handle_time_sync_write()` (`settimeofday`), `handle_weather_write()` (validates length,
stores via `solaris_weather_set_from_ble`, logs). `gatt_svr_subscribe_cb()` /
`gatt_svc_on_disconnect()` manage notify state and reset pagination on disconnect.

### `lib/log_telemetry/` — NVS telemetry snapshot logger
**State: incomplete, unwired, and logically inverted — currently dead code.**

`log_telemetry()` is a 60s-period task that would build a `solaris_telemetry_t` snapshot and
store it to NVS namespace `"solaris_tel"`. Problems:

1. **Never started.** No `xTaskCreate(log_telemetry, ...)` exists anywhere in the repo.
2. **Inverted time-check logic.** The condition and its own comment contradict each other:
   ```c
   // If the recorded year is before 2023 then the ESP likely hasn't synced yet. Don't make a new log
   if (tv.tv_sec < 1700000000) {
       // ... entire logging body executes HERE ...
   }
   ```
   As written, it logs **only when the clock looks unsynced** and skips logging once properly
   time-synced — backwards from the stated intent.
3. **Only ever writes a single record**, overwriting the NVS blob each time rather than
   appending/growing an array, so it can't build a real history even once fixed and started.
4. **TODO** (`log_telemetry.c:30`): `// TODO get implementation to get the telemetry
   information` — precedes `uint8_t battery_level = 0, cpu_temp = 0, net_power_w = 0;`, all
   hardcoded placeholders. Only `record.distance_m = get_distance_traveled();` is real (and
   `get_distance_traveled()` itself is flagged broken — see encoders section).

Corroborated by commit `182c395`: *"added the log telemetry task. It is not completed yet,
but the nvs outline is completed."*

### `lib/motor_driver/` — MCPWM 6-motor driver
**State: functional core logic; PWM duty-cycle math is inverted but internally consistent.**

Motor ID map: 0=Pan, 1=Tilt, 2=FL, 3=FR, 4=RL, 5=RR.

```c
static const motor_pins_t motor_pins[NUM_MOTORS] = {
    {.pwm_gpio = 17, .dir_gpio = 18},  // Pan
    {.pwm_gpio = 21, .dir_gpio = 38},  // Tilt
    {.pwm_gpio = 4,  .dir_gpio = 7},   // FL
    {.pwm_gpio = 6,  .dir_gpio = 9},   // FR
    {.pwm_gpio = 8,  .dir_gpio = 7},   // RL
    {.pwm_gpio = 15, .dir_gpio = 9}};  // RR
```
Note: RL shares `dir_gpio 7` with FL, RR shares `dir_gpio 9` with FR — each side's wheel pair
shares one direction pin (tank-drive wiring where both wheels on a side always spin the same
direction).

- One shared MCPWM timer (group 0, 1MHz resolution, `period_ticks=50` → 20kHz PWM), 3
  operators (each serving 2 motors), each motor with its own comparator + generator.
- `motor_go_forward`/`motor_go_backward` set the shared direction GPIO, then invert the duty
  cycle (`duty_cycle = 1 - duty_cycle`) before scaling to ticks — i.e., passing `0.75`
  actually configures a 25%-on comparator threshold. Panel-motor variants respect soft-limit
  state and refuse to move further past a limit.
- `stop_motor()` resets the comparator to 50 ("stopped"), then does a **blocking 100ms
  `vTaskDelay`** before calling `save_position_to_flash()` for pan/tilt — this runs inline in
  whatever task called `stop_motor`, including `encoder_handler` (priority 18), which stalls
  that task for 100ms each time a limit is hit.
- `test_motor()` — a manual exercise task, never started; most of its body is commented out.
- `panel_get_limit_state`/`panel_set_limit_state` — accessors over `static volatile
  panel_limit_state_t s_state[2]`.

### `lib/nimble_init/` — NimBLE stack bring-up
**State: standard boilerplate, fully functional.**

`nimble_init()` — optional PM config (only if `CONFIG_PM_ENABLE`), `nimble_port_init()`,
conditional `gap_init()`, `gatt_svc_init()`, sets host callbacks (`on_stack_reset`,
`on_stack_sync → adv_init()`, `gatts_register_cb → solaris_gatt_svr_register_cb`,
`store_status_cb`), `ble_store_config_init()`, spawns the NimBLE Host task pinned to core 0.

### `lib/solaris_icm20948/` — IMU driver (ICM-20948 + AK09916 magnetometer)
**State: fully implemented, well-documented — but never called from `main.c` or any task.**

Default config: I2C_NUM_0, SCL=40, SDA=41, 400kHz, addr `0x69`, INT pin=39.

- `solaris_icm20948_init()` — installs I2C driver, verifies `WHO_AM_I` (`0xEA`), wakes device,
  sets `INT_PIN_CFG`, enables raw-data-ready interrupt, probes AK09916 mag at `0x0C` via I2C
  bypass, sets Continuous Measurement Mode 4 (100Hz) if found.
  ⚠️ Comment says `INT_PIN_CFG` should be `0x10 (Clear on Read) + 0x02 (I2C BYPASS ENABLE) =
  0x12`, but the code writes `0x32` — comment/code mismatch, verify against datasheet.
- `solaris_icm20948_read()` — 14 bytes accel+gyro (big-endian, skips 2 temp bytes), 8 bytes
  mag from AK09916 (little-endian + mandatory ST2 read to unlock next measurement).
- `solaris_icm20948_log()`, `solaris_icm20948_deinit()` — as named.

### `lib/solaris_ina228/` — Power/current/SOC monitor (INA228)
**State: fully implemented, thorough docs — but never called anywhere.**

Default config: I2C_NUM_0, SDA=48, SCL=47, 400kHz, addr `0x40`, `current_lsb=1µA`,
`shunt_cal=788`, battery capacity 5000mAh, full=14.4V, empty=12.0V.

- `solaris_ina228_init()` — I2C install, verifies `MANUFACTURER_ID` == `0x5449`, writes
  CONFIG/ADC_CONFIG/SHUNT_CAL.
- Register helpers for 16/24/40-bit big-endian reads, 20-bit sign extension for CURRENT.
- `solaris_ina228_read()` — voltage, current, power, energy (40-bit), charge (40-bit), die
  temp, SOC.
- SOC estimation — voltage-anchored at full/empty (100%/0%), else coulomb-counting from
  CHARGE register scaled by capacity; lazily anchored on first read.
- `solaris_ina228_reset_accumulators()`, `_log()`, `_print_teleplot()` (Serial-Plotter/Teleplot
  compatible debug line).

This is the module intended to feed the shade-detection logic described in `driver.c`'s
design comment (power monitoring as the trigger for automatic-mode shade avoidance) — no
call site exists yet.

### `lib/solaris_manual_ctrl/` — Manual throttle/steering state holder
**State: minimal, functional.**

Trivial globals `g_throttle`/`g_steering` with get/set accessors, no mutex (accessed only
from BLE callback context in this build, so likely fine as-is, but not formally thread-safe).

### `lib/solaris_mock_data/` — Fake historical telemetry generator
**State: complete implementation, never invoked (dev/demo tool).**

`solaris_mock_data_seed()` generates up to 300 synthetic records across 4 time windows
(month/week/day/hour, ~268 total) anchored to "now," using a sine-wave day/night power curve
for `net_power_gain_w` and sine functions of record index for battery/distance/cpu_temp
(not physically meaningful — for UI/demo testing only). Writes to the same NVS namespace
(`"solaris_tel"`) used by `log_telemetry` and read by `solaris_telemetry`. No call site exists
anywhere in the repo — presumably run manually during development/demos.

### `lib/solaris_mode/` — Operating-mode state machine
**State: minimal but complete — the "brain switch" for the 3 top-level behaviors.**

```c
typedef enum {
    SOLARIS_MODE_STATIONARY = 0,
    SOLARIS_MODE_AUTOMATIC  = 1,
    SOLARIS_MODE_MANUAL     = 2,
} solaris_mode_t;
```
Default `g_mode = SOLARIS_MODE_AUTOMATIC`. `solaris_mode_set_from_u8()` validates the raw
byte is ≤ `SOLARIS_MODE_MANUAL`. No mutex protecting `g_mode` (single-byte enum, likely benign
on this MCU but not formally atomic).

### `lib/solaris_telemetry/` — In-RAM telemetry log + BLE pagination backend
**State: fully implemented and actively used; "live telemetry" half is unused.**

```c
typedef struct {
    uint32_t timestamp;
    uint8_t  cpu_temp;
    uint8_t  battery_percent;
    float    distance_m;
    int8_t   net_power_gain_w;
} solaris_telemetry_t; // 11 bytes on the wire (struct itself is padded in memory)
```
`SOLARIS_TELEMETRY_LOG_CAPACITY` = 1440 (in-RAM array, ~17.3KB static RAM);
`SOLARIS_TELEMETRY_RECORDS_PER_PAGE` = 20 (lowered from 30 to keep a page under the BLE MTU
now that `distance_m` is a 4-byte float instead of 1 byte).

- `solaris_telemetry_load()` — reads `count`/`records` blobs from NVS (`"solaris_tel"`) into
  `g_log`.
- `solaris_telemetry_log_count/page_count/get_page()` — pagination helpers consumed by
  `gatt_svc.c`.
- `solaris_telemetry_log_clear()` — resets in-RAM count, erases NVS keys.
- `solaris_telemetry_set/get()` — a separate "live" single-record global (`g_telemetry`),
  distinct from the historical log, intended for "current sensor readings." **Nothing calls
  `solaris_telemetry_set()` anywhere** — this half of the API is currently unpopulated.

### `lib/solaris_ultrasonic/` — 4x MB1020 analog ultrasonic ranging
**State: fully implemented, well-documented — but never called from `main.c` or any task.**

4 sensors via CD4052BE 4-channel analog mux → single ADC pin. Default config: trigger GPIO
20, mux_sel_a=2, mux_sel_b=3, `ADC_UNIT_1`, channel 0, 12-bit, `ADC_ATTEN_DB_12`, 32 samples
averaged, VCC 3300mV.

- `solaris_us_init()` — configures trigger + mux GPIOs, ADC oneshot unit/channel, attempts
  curve-fit calibration (falls back to linear `raw/4095*vcc_mv`).
- `solaris_us_trigger()` — pulses the shared trigger line (all 4 sensors range
  simultaneously).
- `solaris_us_read()` — triggers, waits `ranging_delay_ms` (100ms), reads each mux channel.
  MB1020 scaling: `mv_per_inch = vcc_mv / 512.0f` (per datasheet), converts to inches then cm.
- `solaris_us_log()`/`_print_teleplot()` — debug helpers.

Header explicitly documents ADC-unit coexistence with the phototransistor library (it uses
`ADC_UNIT_1`, deliberately distinct from `solaris_pt`'s `ADC_UNIT_2` — see next section).

### `lib/solaris_weather/` — Weather forecast storage (BLE-fed)
**State: fully functional storage; no consumer logic yet.**

```c
typedef struct {
    uint32_t sunrise, sunset;
    struct { uint32_t time; uint8_t cloud_cover_pct; uint8_t precip_probability_pct; } hourly[24];
} solaris_weather_t; // payload length 152 bytes (4+4+24*6)
```
`solaris_weather_set_from_ble()` validates length and decodes into a static global
(`g_weather`); `solaris_weather_get()` copies it out. **Nothing else reads back
`solaris_weather_get()`** — cloud-cover data is stored but not yet used by any tracking or
driving logic, consistent with `SOLARIS_MODE_AUTOMATIC` being unimplemented (the design
comment in `driver.c` mentions checking "cloud coverage percentage" as one of the shade
signals).

### `lib/solar_tracking/` — Phototransistor pan/tilt sun-tracking task
**State: functional core logic, with one inverted calibration branch.**

⚠️ Header file's Doxygen `@file` tag still says `solaris_phototransistor.h`
(`solar_tracking.h:2`) though the actual filename is `solar_tracking.h` — leftover from an
earlier module name.

8 phototransistors via CD4051B 8-channel mux → single ADC pin, deliberately on `ADC_UNIT_2`
(distinct from `solaris_ultrasonic`'s `ADC_UNIT_1` — documented explicitly in the header's
"COEXISTENCE" section, including the caveat that ADC2 shares hardware with Wi-Fi and reads
can fail if Wi-Fi is active: *"For a standalone tracker this is fine."*).

Sensor index convention: `X1=0, Y1=1, X2=2, Y2=3` (tilt-panel quadrant sensors) and
`FL=4, FR=5, RL=6, RR=7` (chassis-mounted sensors, presumably for the not-yet-implemented
automatic driving/shade logic).

⚠️ **Calibration branch appears inverted** in `prv_raw_to_result()` (`solar_tracking.c:93-103`):
```c
if (!ctx->cali_enabled) {
    // adc_cali_raw_to_voltage(...) -- commented out, does nothing
} else {
    // linear fallback math actually lives here
}
```
Every other ADC module in the repo (`solaris_ultrasonic.c`, `solaris_ina228.c`) follows the
opposite, correct convention (`if (cali_enabled) {calibrated} else {fallback}`). This looks
like a copy-paste bug where the branches got swapped.

The tracking task itself:
```c
void solar_tracking(void *pvParameters)
{
    solaris_pt_result_t result[SOLARIS_PT_MAX_SENSORS / 2];
    int left_right = 0, top_down = 0;
    stop_motor(0); stop_motor(1);
    while (1) {
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
        if (xSemaphoreTake(actuator_mutex, 0) == pdTRUE) {
            solaris_pt_read(pt, result, true);
            left_right = (result[0].mv + result[2].mv) - (result[1].mv + result[3].mv);
            if (left_right > VOLTAGE_TOLERANCE) { /* pan forward or backward */ }
            else if (abs(left_right) > VOLTAGE_TOLERANCE) { /* opposite */ }
            else stop_motor(PANEL_PAN_ID);

            top_down = (result[0].mv + result[1].mv) - (result[2].mv + result[3].mv);
            if (top_down > VOLTAGE_TOLERANCE) motor_go_forward(PANEL_TILT_ID, .15);
            else if (abs(top_down) > VOLTAGE_TOLERANCE) motor_go_backward(PANEL_TILT_ID, .15);
            else stop_motor(PANEL_TILT_ID);
            xSemaphoreGive(actuator_mutex);
        } else { ESP_LOGI(TAG, "Unable to get semaphore"); }
    }
}
```
`VOLTAGE_TOLERANCE` = 500 (mV difference threshold between quadrant sensor pairs before
moving). The task blocks forever on `xTaskNotifyWait` — purely event-driven, not periodic;
it only ever runs when `driver_function` (STATIONARY mode) or anything else with the handle
gives it a notification.

The chassis-direction sensors (indices 4-7) are readable via `solaris_pt_read(handle,
results, false)` but **that code path is never invoked** anywhere — only `true` (tilt/pan
sensors) is ever passed. This is scaffolding for the unimplemented automatic-driving logic.

---

## 4. Hardware Summary

| Subsystem | Interface | Key pins/params |
|---|---|---|
| Pan/Tilt motors | MCPWM, 20kHz | Pan: PWM17/DIR18, Tilt: PWM21/DIR38 |
| Drive motors (FL/FR/RL/RR) | MCPWM, 20kHz | FL: PWM4/DIR7, FR: PWM6/DIR9, RL: PWM8/DIR7, RR: PWM15/DIR9 |
| Pan/Tilt/FL/FR encoders | PCNT | Pan: GPIO37/DIR18, Tilt: GPIO35/DIR38, FL: GPIO42, FR: GPIO45 (⚠️ header comment is stale, see §3) |
| Phototransistors (8x via CD4051B mux) | ADC_UNIT_2, ch0 (GPIO11) | mux sel A/B/C = 10/12/13, 12-bit, 16-sample avg |
| Ultrasonic (4x MB1020 via CD4052BE mux) | ADC_UNIT_1, ch0 | trigger GPIO20, mux sel A/B = 2/3, 32-sample avg |
| IMU (ICM-20948 + AK09916) | I2C_NUM_0 | SDA41/SCL40, addr 0x69, INT39, mag addr 0x0C |
| Power monitor (INA228) | I2C_NUM_0 | SDA48/SCL47, addr 0x40 |

⚠️ **I2C port conflict risk:** ICM-20948 and INA228 both default to `I2C_NUM_0` with
different pin pairs (SDA41/SCL40 vs. SDA48/SCL47). Neither is currently initialized in
`main.c`, so this hasn't surfaced yet — but if both are wired up as-is, the second
`i2c_driver_install()` call on the same port will conflict. Needs reconciling (separate
ports, or a shared I2C bus manager) before these sensors are activated.

---

## 5. What's Actually Wired Up vs. Not (read this before writing a report)

**Working end-to-end today:**
- BLE connect/advertise (GAP), full GATT protocol (telemetry paging, mode set, time sync,
  weather write, manual drive)
- Manual driving via BLE (tank-style mixing)
- Pan/tilt phototransistor sun tracking (STATIONARY mode)
- Motor driver (MCPWM) + encoders (PCNT) + soft limits
- In-RAM telemetry log + NVS load, paginated over BLE

**Implemented in code but never initialized/called from `main.c` or any task (dead code in
this checkout):**
- IMU (`solaris_icm20948`)
- Power monitor (`solaris_ina228`)
- Ultrasonic ranging (`solaris_ultrasonic`)
- `log_telemetry` task (not started — see §3 for the inverted-logic bug too)
- `solaris_mock_data_seed()` (dev/demo utility, no call site)
- Chassis-direction phototransistor read path (`solaris_pt_read(..., false)`)
- `solaris_telemetry_set()` "live telemetry" half of the API
- `test_motor()` task

**Designed but not implemented at all:**
- `SOLARIS_MODE_AUTOMATIC` (the whole shade-avoidance / robot-relocation behavior) — the full
  plan exists as a design comment in `driver.c:10-23` (quoted in full in §3), but the
  switch-case body is empty. Plan summary: detect a sustained power drop via INA228 vs. a 30s/
  3min median comparison, confirm with cloud-cover % from `solaris_weather`, pick a direction
  from the chassis phototransistors, rotate + drive in up to 3 steps via `motor_driver`, and
  check after each step whether the power gain recovered — stop on recovery or after 3 failed
  steps.
- `write_log_and_buffers()` — declared in `driver.h`, never defined.
- Weather-data consumption (stored via BLE, never read back by any logic).

**Known bugs / logic inversions to fix:**
1. `encoders.c` — `load_position_from_flash()` always returns 0; `save_position_to_flash()`
   always saves 0. Panel position is not actually persisted across power cycles.
2. `log_telemetry.c` — time-sync check is inverted; logs only when clock looks unsynced.
3. `solar_tracking.c` — calibration `if`/`else` branches in `prv_raw_to_result()` appear
   swapped relative to every other ADC module in the repo.
4. `solaris_icm20948.c` — `INT_PIN_CFG` write value (`0x32`) doesn't match what the adjacent
   comment computes (`0x12`); verify against the datasheet.
5. ~~`gatt_svc.c` — protocol-doc comment said telemetry records were 9 bytes; actual struct/macro
   size was 8 bytes.~~ Fixed: `distance_m` is now a float, comment/macro both say 11 bytes.
6. `encoders.c` pin-assignment comment is stale relative to the actual `encoder_pins[]` array.
7. `solar_tracking.h` Doxygen `@file` tag says `solaris_phototransistor.h` (old filename).
8. `gap.c` — advertising scan response still uses the Espressif example placeholder URI
   (`https://espressif.com`).

---

## 6. Design Notes / Comments From Development

- Recurring priority: **don't let drive motors and panel motors move at the same time** —
  enforced via `actuator_mutex`, called out explicitly in `shared_resources.h`.
- The FL encoder's high limit (`HIGH_LIMIT_FR = 28387`, distinct from the default ±32767) was
  deliberately chosen "to avoid conflicting interrupts between FL encoder" — a hardware/timing
  workaround, not an arbitrary number.
- ADC unit assignment between the ultrasonic (`ADC_UNIT_1`) and phototransistor
  (`ADC_UNIT_2`) libraries was a deliberate choice to avoid ADC peripheral contention — noted
  explicitly in `solar_tracking.h`'s header comment, along with the Wi-Fi/ADC2 coexistence
  caveat.
- `stop_motor()`'s blocking 100ms delay before flash save was presumably meant to debounce
  rapid stop/start, but it stalls whatever task called it (including the ISR-driven
  `encoder_handler` at priority 18) — worth revisiting if encoder responsiveness becomes an
  issue.
- Git history shows the project moving through: multi-motor init → encoder work → solar
  tracking demo → midterm demo → NVS/telemetry logging groundwork → driving functionality
  (manual + stationary) most recently. Commit messages like *"Will test to see i[f] ISR is
  firing at all"* and *"hopefully done"* reflect an iterative, demo-driven development style
  rather than a fully spec'd build — expect rough edges consistent with an in-progress
  student/hobbyist robotics project rather than a hardened product.

---

## 7. Suggested Next Steps (derived from the above)

1. Fix the two encoder flash-persistence no-ops (§5.1) so panel position actually survives
   reboot.
2. Fix the inverted time-check in `log_telemetry.c`, decide on an append-vs-overwrite storage
   strategy, and actually start the task (or delete it if superseded by `solaris_telemetry`).
3. Wire up IMU / INA228 / ultrasonic in `main.c` — but resolve the I2C_NUM_0 port conflict
   between IMU and INA228 first (separate ports or a shared bus manager).
4. Implement `SOLARIS_MODE_AUTOMATIC` per the full design comment in `driver.c:10-23` (quoted
   in §3): INA228 power ring buffer + 30s/3min median comparison for shade detection, cloud-
   cover % from `solaris_weather` to confirm, chassis phototransistor direction hint, up to 3
   drive steps via `motor_driver`, power-gain recheck after each step. All of INA228, weather
   storage, and the chassis phototransistor read path already exist in the codebase but aren't
   connected to this behavior yet; IMU (heading feedback) and ultrasonic (obstacle sensing) are
   plausible additions but aren't part of the written spec.
5. Fix the calibration branch inversion in `solar_tracking.c`.
6. Verify the ICM-20948 `INT_PIN_CFG` value against the datasheet.
7. Correct the stale encoder pin-assignment comment (the telemetry record doc mismatch is
   fixed).
8. Decide the fate of `write_log_and_buffers()` (`driver.h`) — implement or remove the
   declaration.
