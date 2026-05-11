/**
 * =============================================================================
 * COE411 – Embedded and Cyber Physical Systems
 * Smart Traffic Light System — Milestone 2
 *
 * Team:
 *   Youssef Hussien    (94899)  – FreeRTOS tasks, semaphore/queue wiring
 *   Youssef Abdellatife(94622)  – GPIO/PWM drivers, Wokwi circuit
 *   Seif Hussien       (97215)  – HC-SR04 interfacing, button debounce
 *
 * Platform : ESP32 (Wokwi simulation via PlatformIO / ESP-IDF framework)
 * Scheduler: FreeRTOS with Rate-Monotonic (RM) priorities
 *
 * GPIO Mapping
 * ─────────────────────────────────────────────────────────────────────────────
 *  GPIO 25  – RGB LED Red        (traffic light red)
 *  GPIO 26  – RGB LED Yellow     (traffic light yellow)
 *  GPIO 27  – RGB LED Green      (traffic light green)
 *  GPIO 32  – Buzzer (LEDC PWM)  (pedestrian crossing alert)
 *  GPIO 19  – Push Button        (pedestrian crossing request, pull-up)
 *  GPIO 5   – HC-SR04 TRIG       (ultrasonic pulse trigger)
 *  GPIO 18  – HC-SR04 ECHO       (ultrasonic echo return)
 *  GPIO 21  – LCD SDA            (I2C LCD1602 data)
 *  GPIO 22  – LCD SCL            (I2C LCD1602 clock)
 *
 * FreeRTOS Tasks (RM scheduling — shorter period = higher priority)
 * ─────────────────────────────────────────────────────────────────────────────
 *  Task Name           Priority  Period/Trigger   Purpose
 *  ──────────────────  ────────  ───────────────  ────────────────────────────
 *  Pedestrian Task        4      Aperiodic/ISR    Button press → force red
 *  Traffic Cycle Task     3      1000-10000 ms    Green→Yellow→Red sequence
 *  Ultrasonic Task        2      100 ms           Distance sensing
 *  Buzzer Task            1      Queue-driven     PWM tone generation
 *  Display Task           1      250 ms           LCD1602 countdown + status
 *
 * Inter-task Communication
 * ─────────────────────────────────────────────────────────────────────────────
 *  xSemPedestrian   – Binary semaphore: Pedestrian Task → Traffic Task
 *  xSemExtension    – Binary semaphore: Ultrasonic Task → Traffic Task
 *  xQueueBuzzer     – Queue<BuzzerCmd_t>: Pedestrian/Traffic → Buzzer Task
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_random.h"

/* ─── Logging tags ────────────────────────────────────────────────────────── */
static const char *TAG_TRAFFIC = "TRAFFIC";
static const char *TAG_PED     = "PEDESTRIAN";
static const char *TAG_ULTRA   = "ULTRASONIC";
static const char *TAG_BUZZER  = "BUZZER";

/* ─── GPIO Pin Definitions ────────────────────────────────────────────────── */
#define PIN_LED_RED     GPIO_NUM_25
#define PIN_LED_YELLOW  GPIO_NUM_26
#define PIN_LED_GREEN   GPIO_NUM_27
#define PIN_BUZZER      GPIO_NUM_32
#define PIN_BUTTON      GPIO_NUM_19
#define PIN_TRIG        GPIO_NUM_5
#define PIN_ECHO        GPIO_NUM_18
#define PIN_LCD_SDA     GPIO_NUM_21
#define PIN_LCD_SCL     GPIO_NUM_22

/* raffic Timing (milliseconds) */
#define GREEN_PHASE_MS          8000U
#define YELLOW_PHASE_MS         5000U
#define RED_PHASE_MS            7000U
#define PEDESTRIAN_CROSSING_MS  8000U
#define GREEN_EXTENSION_MS      8000U
#define MAX_EXTENSIONS          3U

/*HC-SR04 Configuration */
#define ULTRA_PERIOD_MS     100U
#define ULTRA_DETECT_CM     20U

/* ─── Buzzer / LEDC Configuration ───────────────────────────────────────────*/
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION     LEDC_TIMER_10_BIT
#define BUZZER_FREQ_PED     1000U
#define BUZZER_FREQ_ALERT   500U
#define BUZZER_DUTY         512U

/* ─── LCD1602 I2C Configuration (PCF8574 backpack, addr 0x27) ────────────── */
#define LCD_I2C_PORT    I2C_NUM_0
#define LCD_I2C_ADDR    0x27
#define LCD_I2C_FREQ    100000
#define LCD_BL          0x08
#define LCD_EN          0x04
#define LCD_RW          0x02
#define LCD_RS          0x01

/* ─── Buzzer Command Queue ────────────────────────────────────────────────── */
typedef enum {
    CMD_PEDESTRIAN_BEEP = 0,
    CMD_ALERT_TONE
} BuzzerCmd_t;

#define BUZZER_QUEUE_LEN 5U

/* ─── Traffic State ───────────────────────────────────────────────────────── */
typedef enum { TS_GREEN = 0, TS_YELLOW, TS_RED } TrafficState_t;

/* ─── Status Messages (shown on LCD row 1) ───────────────────────────────── */
typedef enum {
    ST_NORMAL = 0,   /* Normal cycle in progress  */
    ST_PED_REQ,      /* Pedestrian button pressed  */
    ST_PED_CROSS,    /* Pedestrian crossing active */
    ST_VEHICLE,      /* Vehicle detected by sensor */
    ST_EXTENDED,     /* Green phase extended       */
    ST_CAP_FORCED,   /* Max extensions → yellow   */
    ST_INTERRUPTED,  /* Ped request cut green      */
    ST_COUNT
} Status_t;

/* All strings are exactly 16 chars so no clearing is ever needed */
static const char *const STATUS_STR[ST_COUNT] = {
    "Normal Cycle    ",
    "Pedestrian Req! ",
    "Pedestrian Pass ",
    "Vehicle Detect! ",
    "Green Extended! ",
    "Max Ext->Yellow ",
    "Ped Interrupted!",
};

/* ─── FreeRTOS Handles ────────────────────────────────────────────────────── */
static SemaphoreHandle_t xSemPedestrian = NULL;
static SemaphoreHandle_t xSemExtension  = NULL;
static QueueHandle_t     xQueueBuzzer   = NULL;

/* ─── Shared State (single writer per variable) ───────────────────────────── */
static volatile TrafficState_t g_state        = TS_RED;
static volatile Status_t       g_status       = ST_NORMAL;
static volatile uint32_t       g_remaining_ms = 0;
static volatile bool           g_vehicle_near = false;

/* ═══════════════════════════════════════════════════════════════════════════
 * LCD1602 I2C DRIVER  (HD44780 via PCF8574 backpack, 4-bit mode)
 *
 * All three PCF8574 bytes for one nibble are sent in a single I2C burst:
 *   [d, EN=0]  →  [d, EN=1]  →  [d, EN=0]
 * At 100 kHz each byte takes ~90 µs, which comfortably satisfies the
 * HD44780's 450 ns EN-pulse and 37 µs settle requirements.
 * No esp_rom_delay_us is used — Wokwi's simulated time makes those
 * busy-wait loops unreliable.
 * ═══════════════════════════════════════════════════════════════════════════ */

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

static void lcd_cmd(uint8_t cmd)
{
    lcd_send_nibble(cmd >> 4,   0);
    lcd_send_nibble(cmd & 0x0F, 0);
}

static void lcd_char(uint8_t ch)
{
    lcd_send_nibble(ch >> 4,   LCD_RS);
    lcd_send_nibble(ch & 0x0F, LCD_RS);
}

static void lcd_puts(const char *s)
{
    while (*s) lcd_char((uint8_t)*s++);
}

static void lcd_goto(uint8_t col, uint8_t row)
{
    lcd_cmd(0x80 | (col + (row ? 0x40 : 0x00)));
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));          /* >40 ms power-on wait      */
    lcd_send_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));           /* >4.1 ms                   */
    lcd_send_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(2));           /* >100 µs                   */
    lcd_send_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(2));           /* >100 µs                   */
    lcd_send_nibble(0x02, 0);               /* switch to 4-bit           */
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_cmd(0x28);  /* 2 lines, 5×8 font */
    lcd_cmd(0x0C);  /* display on, cursor off */
    lcd_cmd(0x06);  /* auto-increment cursor */
    lcd_cmd(0x01);  /* clear display */
    vTaskDelay(pdMS_TO_TICKS(2));           /* >1.52 ms clear delay      */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE INITIALISATION
 * ═══════════════════════════════════════════════════════════════════════════ */

static void gpio_init_all(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << PIN_LED_RED) |
                        (1ULL << PIN_LED_YELLOW) |
                        (1ULL << PIN_LED_GREEN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);

    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << PIN_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << PIN_ECHO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_conf);

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    gpio_set_level(PIN_LED_RED,    0);
    gpio_set_level(PIN_LED_YELLOW, 0);
    gpio_set_level(PIN_LED_GREEN,  0);
    gpio_set_level(PIN_TRIG,       0);
}

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_LCD_SDA,
        .scl_io_num       = PIN_LCD_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_I2C_FREQ,
    };
    i2c_param_config(LCD_I2C_PORT, &conf);
    i2c_driver_install(LCD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void buzzer_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = BUZZER_FREQ_PED,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = PIN_BUZZER,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_cfg);
}

/* ─── LED helpers ─────────────────────────────────────────────────────────── */
static inline void set_led_red(void)
{
    gpio_set_level(PIN_LED_RED,    1);
    gpio_set_level(PIN_LED_YELLOW, 0);
    gpio_set_level(PIN_LED_GREEN,  0);
}

static inline void set_led_yellow(void)
{
    gpio_set_level(PIN_LED_RED,    0);
    gpio_set_level(PIN_LED_YELLOW, 1);
    gpio_set_level(PIN_LED_GREEN,  0);
}

static inline void set_led_green(void)
{
    gpio_set_level(PIN_LED_RED,    0);
    gpio_set_level(PIN_LED_YELLOW, 0);
    gpio_set_level(PIN_LED_GREEN,  1);
}

/* ─── Buzzer helper ───────────────────────────────────────────────────────── */
static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, BUZZER_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/* ─── Countdown helper: delays for `total_ms`, updating g_remaining_ms ─────
 * Polls every POLL_MS, allowing the display task to show a live countdown.  */
#define COUNTDOWN_POLL_MS 50U

static void countdown_delay(uint32_t total_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t total = pdMS_TO_TICKS(total_ms);
    while (1) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        TickType_t left    = (elapsed < total) ? (total - elapsed) : 0;
        g_remaining_ms     = (uint32_t)pdTICKS_TO_MS(left);
        if (elapsed >= total) break;
        vTaskDelay(pdMS_TO_TICKS(COUNTDOWN_POLL_MS));
    }
    g_remaining_ms = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK: TRAFFIC CYCLE TASK  (Priority 3)
 *
 * Drives Green → Yellow → Red sequence.
 * Updates g_state, g_status, and g_remaining_ms throughout.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void task_traffic_cycle(void *pvParameters)
{
    (void)pvParameters;
    BuzzerCmd_t cmd;

    ESP_LOGI(TAG_TRAFFIC, "Traffic Cycle Task started");

    while (1) {
        /* ── GREEN ───────────────────────────────────────────────────────── */
        set_led_green();
        g_state  = TS_GREEN;
        g_status = ST_NORMAL;
        ESP_LOGI(TAG_TRAFFIC, "GREEN – %u ms", GREEN_PHASE_MS);

        uint32_t   extensions  = 0;
        TickType_t green_start = xTaskGetTickCount();
        TickType_t green_total = pdMS_TO_TICKS(GREEN_PHASE_MS);

        while (1) {
            TickType_t elapsed = xTaskGetTickCount() - green_start;
            TickType_t left    = (elapsed < green_total) ? (green_total - elapsed) : 0;
            g_remaining_ms     = (uint32_t)pdTICKS_TO_MS(left);

            /* Pedestrian request — highest priority, jump to yellow immediately */
            if (xSemaphoreTake(xSemPedestrian, 0) == pdTRUE) {
                ESP_LOGI(TAG_TRAFFIC, "GREEN interrupted by pedestrian");
                g_remaining_ms = 0;
                g_status       = ST_INTERRUPTED;
                goto do_yellow;
            }

            /* Vehicle-triggered extension */
            if (xSemaphoreTake(xSemExtension, 0) == pdTRUE) {
                if (extensions < MAX_EXTENSIONS) {
                    extensions++;
                    green_total += pdMS_TO_TICKS(GREEN_EXTENSION_MS);
                    g_status = ST_EXTENDED;
                    ESP_LOGI(TAG_TRAFFIC, "GREEN extended (ext %u/%u)",
                             extensions, MAX_EXTENSIONS);
                } else {
                    ESP_LOGW(TAG_TRAFFIC, "Max extensions – forcing yellow");
                    g_remaining_ms = 0;
                    g_status       = ST_CAP_FORCED;
                    goto do_yellow;
                }
            }

            if (elapsed >= green_total) break;
            vTaskDelay(pdMS_TO_TICKS(COUNTDOWN_POLL_MS));
        }
        g_remaining_ms = 0;

        /* ── YELLOW ──────────────────────────────────────────────────────── */
do_yellow:
        set_led_yellow();
        g_state = TS_YELLOW;
        ESP_LOGI(TAG_TRAFFIC, "YELLOW – %u ms", YELLOW_PHASE_MS);
        countdown_delay(YELLOW_PHASE_MS);

        /* ── RED ─────────────────────────────────────────────────────────── */
        set_led_red();
        g_state = TS_RED;
        ESP_LOGI(TAG_TRAFFIC, "RED – %u ms", RED_PHASE_MS);

        bool ped_crossing = (xSemaphoreTake(xSemPedestrian, 0) == pdTRUE);
        if (ped_crossing) {
            ESP_LOGI(TAG_TRAFFIC, "Pedestrian crossing active");
            g_status = ST_PED_CROSS;
            cmd = CMD_PEDESTRIAN_BEEP;
            xQueueSend(xQueueBuzzer, &cmd, 0);
            countdown_delay(RED_PHASE_MS + PEDESTRIAN_CROSSING_MS);
        } else {
            g_status = ST_NORMAL;
            countdown_delay(RED_PHASE_MS);
        }

        /* Drain stale pedestrian semaphore before next green */
        xSemaphoreTake(xSemPedestrian, 0);
        g_status = ST_NORMAL;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK: PEDESTRIAN TASK  (Priority 4 – highest)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void task_pedestrian(void *pvParameters)
{
    (void)pvParameters;
    bool        button_armed = true;
    BuzzerCmd_t cmd = CMD_PEDESTRIAN_BEEP;

    ESP_LOGI(TAG_PED, "Pedestrian Task started");

    while (1) {
        if (gpio_get_level(PIN_BUTTON) == 0 && button_armed) {
            vTaskDelay(pdMS_TO_TICKS(20));  /* debounce */
            if (gpio_get_level(PIN_BUTTON) == 0) {
                button_armed = false;
                ESP_LOGI(TAG_PED, "Button pressed – pedestrian request");
                g_status = ST_PED_REQ;
                xSemaphoreGive(xSemPedestrian);
                xQueueSend(xQueueBuzzer, &cmd, 0);
            }
        } else if (gpio_get_level(PIN_BUTTON) == 1) {
            button_armed = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


 /* SIMULATED CAR DETECTION
 *
 * The HC-SR04 echo pin never triggers in Wokwi when no physical object is
 * present, so we simulate random vehicle arrivals here instead.
 *
 * Model:
 *   - Each 100 ms poll there is a CAR_ARRIVE_PCT% chance a car "arrives".
 *   - Once present it stays for CAR_MIN_POLLS … CAR_MAX_POLLS polls
 *     (i.e. CAR_MIN_POLLS*100 ms … CAR_MAX_POLLS*100 ms).
 *   - While a car is present, distance = 5–14 cm (well inside the 20 cm
 *     detection threshold).  Otherwise distance = 80–119 cm (clear).
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CAR_ARRIVE_PCT   40U  /* % chance per poll once gap expires       */
#define CAR_MIN_POLLS    15U  /* minimum presence: 1.5 s                  */
#define CAR_MAX_POLLS    35U  /* maximum presence: 3.5 s                  */
#define CAR_MIN_GAP_POLLS 50U /* minimum gap between cars: 5 s            */
#define CAR_MAX_GAP_POLLS 150U/* maximum gap between cars: 15 s           */

static uint32_t s_car_polls_left = 0;
static uint32_t s_car_gap_left   = 0; /* cooldown before next car may arrive */
static bool     s_ext_given      = false; /* one extension per car visit */

static uint32_t sim_distance_cm(void)
{
    /* Car currently present */
    if (s_car_polls_left > 0) {
        s_car_polls_left--;
        if (s_car_polls_left == 0) {
            /* Car just left — enforce a random gap before the next one */
            s_car_gap_left = CAR_MIN_GAP_POLLS +
                             (esp_random() % (CAR_MAX_GAP_POLLS - CAR_MIN_GAP_POLLS + 1));
        }
        return 5 + (esp_random() % 10);                     /* 5–14 cm  */
    }
    /* Mandatory quiet period after the last car */
    if (s_car_gap_left > 0) {
        s_car_gap_left--;
        return 80 + (esp_random() % 40);                    /* 80–119 cm */
    }
    /* Gap expired — roll for a new arrival */
    if ((esp_random() % 100) < CAR_ARRIVE_PCT) {
        s_car_polls_left = CAR_MIN_POLLS +
                           (esp_random() % (CAR_MAX_POLLS - CAR_MIN_POLLS + 1));
        return 5 + (esp_random() % 10);                     /* 5–14 cm  */
    }
    return 80 + (esp_random() % 40);                        /* 80–119 cm */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK: ULTRASONIC TASK  (Priority 2)
 *
 * Fires the TRIG pulse as normal, then uses sim_distance_cm() in place of
 * the real echo measurement (echo never fires in Wokwi without a target).
 * ═══════════════════════════════════════════════════════════════════════════ */
static void task_ultrasonic(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG_ULTRA, "Ultrasonic Task started (car simulation active)");

    while (1) {
        /* Fire the trigger pulse (keeps hardware behaviour authentic) */
        gpio_set_level(PIN_TRIG, 0);
        esp_rom_delay_us(2);
        gpio_set_level(PIN_TRIG, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TRIG, 0);

        uint32_t distance_cm = sim_distance_cm();
        ESP_LOGD(TAG_ULTRA, "Simulated distance: %u cm", distance_cm);

        if (distance_cm < ULTRA_DETECT_CM) {
            g_vehicle_near = true;
            if (g_state == TS_GREEN && !s_ext_given) {
                s_ext_given = true;   // block further requests until car leaves
                if (g_status == ST_NORMAL || g_status == ST_VEHICLE) {
                    g_status = ST_VEHICLE;
                }
                ESP_LOGI(TAG_ULTRA, "Vehicle at %u cm – extension request", distance_cm);
                xSemaphoreGive(xSemExtension);
            }
        } else {
            g_vehicle_near = false;
            s_ext_given = false;      /* car gone — next arrival may request again */
            if (g_status == ST_VEHICLE) g_status = ST_NORMAL;
        }

        vTaskDelay(pdMS_TO_TICKS(ULTRA_PERIOD_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK: BUZZER TASK  (Priority 1)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void task_buzzer(void *pvParameters)
{
    (void)pvParameters;
    BuzzerCmd_t cmd;

    ESP_LOGI(TAG_BUZZER, "Buzzer Task started");

    while (1) {
        if (xQueueReceive(xQueueBuzzer, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd) {
                case CMD_PEDESTRIAN_BEEP:
                    ESP_LOGI(TAG_BUZZER, "Pedestrian crossing beeps");
                    for (int i = 0; i < 3; i++) {
                        buzzer_tone(BUZZER_FREQ_PED, 200);
                        vTaskDelay(pdMS_TO_TICKS(150));
                    }
                    break;
                case CMD_ALERT_TONE:
                    ESP_LOGI(TAG_BUZZER, "Alert tone");
                    buzzer_tone(BUZZER_FREQ_ALERT, 1000);
                    break;
                default:
                    break;
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK: DISPLAY TASK  (Priority 1, 250 ms period)
 *
 * LCD row 0:  "GREEN      20s  "   (state left, countdown right)
 * LCD row 1:  "Normal Cycle    "   (16-char status string)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void task_display(void *pvParameters)
{
    (void)pvParameters;
    char row0[32];  /* generous buffer; LCD only shows first 16 chars */

    ESP_LOGI("DISPLAY", "Display Task started");

    while (1) {
        TrafficState_t state  = g_state;
        Status_t       status = g_status;
        uint32_t       rem_s  = (g_remaining_ms + 999) / 1000;  /* round up */

        const char *state_str;
        switch (state) {
            case TS_GREEN:  state_str = "GREEN";  break;
            case TS_YELLOW: state_str = "YELLOW"; break;
            default:        state_str = "RED";    break;
        }

        /*
         * Row 0 format (16 chars visible):
         *   "%-6s   %2us      "
         *   col 0–5  : state name, left-aligned, padded to 6
         *   col 6–8  : spaces
         *   col 9–11 : seconds right-justified in 2 + "s"
         *   col 12+  : trailing spaces (off-screen on LCD)
         */
        snprintf(row0, sizeof(row0), "%-6s   %2us      ", state_str, (unsigned)rem_s);

        lcd_goto(0, 0); lcd_puts(row0);
        lcd_goto(0, 1); lcd_puts(STATUS_STR[status]);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * APP_MAIN
 * ═══════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI("MAIN", "Smart Traffic Light System – COE411 Milestone 2");

    gpio_init_all();
    buzzer_init();
    i2c_init();
    lcd_init();

    /* Splash screen */
    lcd_goto(0, 0); lcd_puts("Traffic Light   ");
    lcd_goto(0, 1); lcd_puts("COE411 Starting ");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));

    xSemPedestrian = xSemaphoreCreateBinary();
    xSemExtension  = xSemaphoreCreateBinary();
    xQueueBuzzer   = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(BuzzerCmd_t));

    if (!xSemPedestrian || !xSemExtension || !xQueueBuzzer) {
        ESP_LOGE("MAIN", "FreeRTOS object creation failed – halting");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    xTaskCreate(task_traffic_cycle, "TrafficCycle", 4096, NULL, 3, NULL);
    xTaskCreate(task_pedestrian,    "Pedestrian",   2048, NULL, 4, NULL);
    xTaskCreate(task_ultrasonic,    "Ultrasonic",   2048, NULL, 2, NULL);
    xTaskCreate(task_buzzer,        "Buzzer",       2048, NULL, 1, NULL);
    xTaskCreate(task_display,       "Display",      3072, NULL, 1, NULL);

    ESP_LOGI("MAIN", "All tasks running");
}
