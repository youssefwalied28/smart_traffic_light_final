# Smart Traffic Light — Development Log

**Course:** COE411 – Embedded and Cyber Physical Systems  
**Team:**
- Youssef Hussien (94899)
- Youssef Abdellatife (94622)
- Seif Hussien (97215)

**Platform:** ESP32 · PlatformIO · ESP-IDF · Wokwi simulation

---

## Session Summary

This document records the issues encountered and fixes applied during a single development session, from getting the project to build through adding and debugging the LCD status display and car simulation.

---

## 1. Project Would Not Run

### Problem

Clicking "Run Simulation" in Wokwi produced nothing. The simulation started but the firmware was never loaded.

### Root Cause

The firmware had never been compiled. Wokwi reads `firmware.elf` and `firmware.bin` from `.pio/build/esp32dev/`, but that directory was empty.

### Fix — Build via a space-free path

PlatformIO refuses to build from any path that contains spaces. The project lives inside:

```
/Users/seif/Documents/AUS/Senior/Spring 2026/COE 411/COE 411L/smart_traffic_light
```

Both `COE 411` and `COE 411L` contain spaces, so a direct `pio run` fails with:

```
Error: Detected a whitespace character in project paths.
```

**Workaround:** copy the project to a temporary path without spaces, build there, then copy the firmware back.

```bash
cp -r "<project path>" /tmp/smart_tl
cd /tmp/smart_tl
~/.platformio/penv/bin/pio run
cp .pio/build/esp32dev/firmware.{elf,bin} "<project path>/.pio/build/esp32dev/"
```

Use this same sequence after every code change. The permanent fix is to move the project folder to a path with no spaces.

---

## 2. Adding an LCD Status Screen

### Requirement

Add a 16×2 LCD that shows the current traffic light state and system status.

### Hardware Added (`diagram.json`)

A `wokwi-lcd1602` in I2C mode (PCF8574 backpack, address `0x27`) was added and wired to:

| Signal | ESP32 GPIO |
|--------|-----------|
| SDA    | GPIO 21   |
| SCL    | GPIO 22   |
| VCC    | 3V3       |
| GND    | GND       |

### Software Added (`src/main.c`)

**I2C init**

```c
i2c_config_t conf = {
    .mode             = I2C_MODE_MASTER,
    .sda_io_num       = GPIO_NUM_21,
    .scl_io_num       = GPIO_NUM_22,
    .sda_pullup_en    = GPIO_PULLUP_ENABLE,
    .scl_pullup_en    = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};
i2c_param_config(I2C_NUM_0, &conf);
i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
```

**New FreeRTOS task — `task_display` (priority 1, 250 ms period)**

- Row 0: current light colour + countdown seconds (`GREEN     8s`)
- Row 1: 16-char status string (see table below)

**Shared state variables** (volatile, single writer per variable):

| Variable | Writer | Purpose |
|----------|--------|---------|
| `g_state` | `task_traffic_cycle` | Current phase (GREEN/YELLOW/RED) |
| `g_status` | all tasks | Status enum for LCD row 1 |
| `g_remaining_ms` | `task_traffic_cycle` | Milliseconds left in current phase |
| `g_vehicle_near` | `task_ultrasonic` | Whether a vehicle is currently detected |

**Status messages displayed on row 1:**

| Enum value | LCD text |
|------------|----------|
| `ST_NORMAL` | `Normal Cycle    ` |
| `ST_PED_REQ` | `Pedestrian Req! ` |
| `ST_PED_CROSS` | `Pedestrian Pass ` |
| `ST_VEHICLE` | `Vehicle Detect! ` |
| `ST_EXTENDED` | `Green Extended! ` |
| `ST_CAP_FORCED` | `Max Ext->Yellow ` |
| `ST_INTERRUPTED` | `Ped Interrupted!` |

**Countdown tracking**

Single-`vTaskDelay` calls for yellow and red phases were replaced with a `countdown_delay()` helper that polls every 50 ms and writes `g_remaining_ms` continuously, giving the display task a live countdown to show.

```c
static void countdown_delay(uint32_t total_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t total = pdMS_TO_TICKS(total_ms);
    while (1) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        TickType_t left    = (elapsed < total) ? (total - elapsed) : 0;
        g_remaining_ms     = (uint32_t)pdTICKS_TO_MS(left);
        if (elapsed >= total) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    g_remaining_ms = 0;
}
```

---

## 3. LCD Showing Garbled Symbols

### Problem

The LCD displayed random symbols and blocks instead of English text.

### Root Cause

The original LCD driver used `esp_rom_delay_us()` to generate the HD44780 EN-pulse timing. This function is a CPU busy-loop that counts clock cycles. In Wokwi's virtual simulation, simulated time does not advance during a busy-loop, so the EN pulse appeared instantaneous and the LCD never latched any data — leaving it in its random power-on DDRAM state.

### Fix — Single I2C burst per nibble

Instead of three separate I2C transactions with `esp_rom_delay_us` between them, all three PCF8574 bytes for one nibble are sent in a **single I2C transaction**:

```
[data, EN=0]  →  [data, EN=1]  →  [data, EN=0]
```

At 100 kHz, each I2C byte takes ~90 µs. The EN pulse is therefore held high for a full byte-time (~90 µs), which comfortably satisfies the HD44780's 450 ns minimum EN-pulse requirement — with no `esp_rom_delay_us` at all.

```c
static void lcd_burst(uint8_t b0, uint8_t b1, uint8_t b2)
{
    uint8_t buf[3] = {b0, b1, b2};
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(h, buf, 3, true);
    i2c_master_stop(h);
    i2c_master_cmd_begin(LCD_I2C_PORT, h, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(h);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t d = (nibble << 4) | LCD_BL | rs;
    lcd_burst(d, d | LCD_EN, d);
}
```

All initialization delays were also changed from `esp_rom_delay_us` to `vTaskDelay`, which properly advances simulation time.

---

## 4. Car Simulation (HC-SR04 Never Triggers in Wokwi)

### Problem

The HC-SR04 ECHO pin never goes high in Wokwi without a physical target object. Without car detections the green-extension feature could not be demonstrated.

### Fix — Random car arrival model

The real echo measurement was replaced with `sim_distance_cm()`, a function that models random vehicle arrivals using ESP32's hardware RNG (`esp_random()`):

```
Parameters:
  CAR_ARRIVE_PCT  = 5   (5% chance per 100 ms poll ≈ one car every ~2 s)
  CAR_MIN_POLLS   = 15  (car stays at least 1.5 s)
  CAR_MAX_POLLS   = 35  (car stays at most 3.5 s)

While car present : distance = 5–14 cm  (inside 20 cm threshold)
While road clear  : distance = 80–119 cm (outside threshold)
```

The TRIG pulse is still fired every cycle so the sensor waveform looks authentic in Wokwi's logic analyser.

---

## 5. All Extensions Firing Instantly

### Problem

The serial monitor showed all three green extensions granted within the same second:

```
I (8783) ULTRASONIC: Vehicle at 13 cm – extension request
I (8833) TRAFFIC: GREEN extended (ext 1/3)
I (8883) ULTRASONIC: Vehicle at 10 cm – extension request
I (8933) TRAFFIC: GREEN extended (ext 2/3)
I (8983) ULTRASONIC: Vehicle at 11 cm – extension request
I (9033) TRAFFIC: GREEN extended (ext 3/3)
W (9133) TRAFFIC: Max extensions – forcing yellow
```

### Root Cause

`task_ultrasonic` polls every 100 ms. While `s_car_polls_left > 0` the simulated car is continuously "present", so `xSemaphoreGive(xSemExtension)` was called on **every** poll. The traffic task consumed each semaphore within 50 ms, granting all three extensions in rapid succession.

### Fix — One extension per car visit

A boolean flag `s_ext_given` was added. It is set to `true` the moment the car triggers an extension request, and only resets to `false` when the distance rises back above the threshold (car leaves). No matter how many polls the car is present for, it generates exactly one extension request per visit.

```c
static bool s_ext_given = false;

// In task_ultrasonic:
if (distance_cm < ULTRA_DETECT_CM) {
    g_vehicle_near = true;
    if (g_state == TS_GREEN && !s_ext_given) {
        s_ext_given = true;
        xSemaphoreGive(xSemExtension);
    }
} else {
    g_vehicle_near = false;
    s_ext_given = false;   // reset — next car can request again
}
```

---

## 6. Timing Adjustment

All phase durations were increased by 3 seconds to make the simulation easier to follow:

| Phase | Original | Final |
|-------|----------|-------|
| Green | 5 s | 8 s |
| Yellow | 2 s | 5 s |
| Red | 4 s | 7 s |
| Pedestrian crossing | 5 s | 8 s |
| Green extension | 5 s | 8 s |

---

## Final System Overview

```
GPIO 25  Red LED   ─── 68 Ω ──── LED ──── GND
GPIO 26  Yellow LED─── 68 Ω ──── LED ──── GND
GPIO 27  Green LED ─── 68 Ω ──── LED ──── GND
GPIO 32  Buzzer (LEDC PWM)
GPIO 19  Push button (pull-up, active-low)
GPIO 5   HC-SR04 TRIG
GPIO 18  HC-SR04 ECHO
GPIO 21  LCD SDA (I2C, 100 kHz)
GPIO 22  LCD SCL
```

```
FreeRTOS Tasks
──────────────────────────────────────────────────────
task_pedestrian   priority 4   10 ms poll    button debounce
task_traffic_cycle priority 3  event-driven  GREEN→YELLOW→RED
task_ultrasonic   priority 2   100 ms poll   car simulation
task_buzzer       priority 1   queue-blocked PWM tones
task_display      priority 1   250 ms poll   LCD countdown + status
```
