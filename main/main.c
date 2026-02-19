#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ─── Config ──────────────────────────────────────────────────────────────────

#define SIGNAL_GPIO         GPIO_NUM_27  // Hall sensor input
#define LED_GPIO            GPIO_NUM_4   // Addressable LED data pin
#define LED_STRIP_LENGTH    1            // Number of LEDs
#define TIMER_RESOLUTION_HZ 40000000     // 40 MHz → 25 ns per tick (max for gptimer)

#define UART_PORT           UART_NUM_0
#define UART_BAUD           115200
#define UART_TX_PIN         GPIO_NUM_1
#define UART_RX_PIN         GPIO_NUM_3
#define UART_BUF_SIZE       256

#define RING_BUF_SIZE       16
#define RING_BUF_MASK       (RING_BUF_SIZE - 1)

#define LED_PULSE_MS        20           // LED pulse duration in milliseconds
#define TIMEOUT_MS          2200         // Timeout for detecting stopped wheel (2.2 seconds)

// Missing magnet detection threshold
#define ANOMALY_THRESHOLD   1.5f         // If a pulse is 1.5x longer than average, flag it

// Bicycle wheel configuration
#define MAGNETS_PER_REV     4            // Number of magnets on the wheel
#define WHEEL_DIAMETER_MM   700          // Wheel diameter in millimeters

// ─── Ring buffers ────────────────────────────────────────────────────────────

typedef struct {
    uint64_t data[RING_BUF_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} ring_buf_t;

// Ring buffer for direct method (consecutive pulse differences)
static ring_buf_t ring_direct = {0};

// Ring buffer for universal counter method (alternating measurement windows)
static ring_buf_t ring_counter = {0};

static inline bool IRAM_ATTR ring_push(ring_buf_t *rb, uint64_t value)
{
    uint32_t next = (rb->write_idx + 1) & RING_BUF_MASK;
    if (next == rb->read_idx) {
        return false;
    }
    rb->data[rb->write_idx] = value;
    rb->write_idx = next;
    return true;
}

static inline bool ring_pop(ring_buf_t *rb, uint64_t *out)
{
    if (rb->read_idx == rb->write_idx) {
        return false;
    }
    *out = rb->data[rb->read_idx];
    rb->read_idx = (rb->read_idx + 1) & RING_BUF_MASK;
    return true;
}

// ─── Globals ─────────────────────────────────────────────────────────────────

static gptimer_handle_t gptimer = NULL;
static led_strip_handle_t led_strip = NULL;

// Direct measurement state
static volatile uint64_t last_timestamp_direct = 0;
static volatile bool     first_edge_direct     = true;

// Universal counter state
static volatile uint64_t window_start = 0;       // Timestamp when counting window opened
static volatile uint64_t calculated_period = 0;  // Period calculated during previous window
static volatile uint8_t  pulse_count = 0;        // Counts pulses: 0, 1, 2, 3, 0, 1, 2, 3...
static volatile bool     first_edge_counter = true;

static volatile bool     trigger_led    = false;

// Timeout detection
static volatile TickType_t last_pulse_time = 0;  // FreeRTOS tick when last pulse arrived

// Moving average for anomaly detection
#define HISTORY_SIZE 8
static uint64_t period_history[HISTORY_SIZE] = {0};
static uint8_t history_index = 0;
static uint8_t history_count = 0;

// ─── ISR ─────────────────────────────────────────────────────────────────────

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint64_t now = 0;
    gptimer_get_raw_count(gptimer, &now);

    // Update last pulse time for timeout detection
    last_pulse_time = xTaskGetTickCountFromISR();

    // ─── Method 1: Direct consecutive pulse measurement ─────────────────────
    if (first_edge_direct) {
        first_edge_direct = false;
    } else {
        ring_push(&ring_direct, now - last_timestamp_direct);
    }
    last_timestamp_direct = now;

    // ─── Method 2: Universal counter (alternating windows) ──────────────────
    if (first_edge_counter) {
        // Very first pulse ever - just initialize
        window_start = now;
        pulse_count = 1;
        first_edge_counter = false;
    } else {
        pulse_count++;
        
        if (pulse_count == 2) {
            // Second pulse: close the counting window and calculate period
            calculated_period = now - window_start;
        } else if (pulse_count == 3) {
            // Third pulse: push the calculated period and open new window
            ring_push(&ring_counter, calculated_period);
            window_start = now;
            pulse_count = 1;  // Reset for next window (this is pulse 1 of next window)
        }
    }

    trigger_led = true;
}

// ─── UART init ───────────────────────────────────────────────────────────────

static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0));
}

// ─── Timer init ──────────────────────────────────────────────────────────────

static void init_timer(void)
{
    gptimer_config_t timer_config = {
        .clk_src       = GPTIMER_CLK_SRC_APB,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

// ─── LED strip init ──────────────────────────────────────────────────────────

static void init_led_strip(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

// ─── GPIO init ───────────────────────────────────────────────────────────────

static void init_gpio(void)
{
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << SIGNAL_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SIGNAL_GPIO, gpio_isr_handler, NULL));
}

// ─── LED control task ────────────────────────────────────────────────────────

static void led_task(void *arg)
{
    TickType_t led_off_time = 0;
    bool led_is_on = false;

    while (1) {
        if (trigger_led) {
            trigger_led = false;
            
            led_strip_set_pixel(led_strip, 0, 0, 255, 0);  // Green
            led_strip_refresh(led_strip);
            
            led_is_on = true;
            led_off_time = xTaskGetTickCount() + pdMS_TO_TICKS(LED_PULSE_MS);
        }

        if (led_is_on && (xTaskGetTickCount() >= led_off_time)) {
            led_strip_clear(led_strip);
            led_is_on = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Anomaly detection ───────────────────────────────────────────────────────

static void update_period_history(uint64_t period)
{
    period_history[history_index] = period;
    history_index = (history_index + 1) % HISTORY_SIZE;
    if (history_count < HISTORY_SIZE) {
        history_count++;
    }
}

static float get_average_period(void)
{
    if (history_count == 0) return 0.0f;
    
    uint64_t sum = 0;
    for (uint8_t i = 0; i < history_count; i++) {
        sum += period_history[i];
    }
    return (float)sum / (float)history_count;
}

static bool detect_anomaly(uint64_t current_period, float* corrected_period)
{
    if (history_count < 3) {
        // Need at least 3 samples to detect anomalies
        return false;
    }
    
    float avg = get_average_period();
    float ratio = (float)current_period / avg;
    
    // If current period is significantly longer than average, it's likely a missing magnet
    if (ratio > ANOMALY_THRESHOLD) {
        // Estimate how many magnets are missing and calculate corrected period
        // For example, if ratio is ~2.0, likely 1 magnet missing (should be 2 magnets, not 1)
        // Corrected period = current_period / ratio (approximately equal to average)
        *corrected_period = avg;
        return true;
    }
    
    return false;
}

// ─── Helper function for speed/RPM calculation ───────────────────────────────

static void calculate_and_print(uint64_t ticks, const char* method_name, float wheel_circumference_m, bool is_anomaly, float corrected_ticks)
{
    char buf[200];
    
    if (is_anomaly) {
        // Use corrected period for calculation
        float period_s = corrected_ticks / (float)TIMER_RESOLUTION_HZ;
        float time_per_rev_s = period_s * MAGNETS_PER_REV;
        float rpm = 60.0f / time_per_rev_s;
        
        float distance_per_pulse_m = wheel_circumference_m / MAGNETS_PER_REV;
        float speed_ms = distance_per_pulse_m / period_s;
        float speed_kmh = speed_ms * 3.6f;

        int len = snprintf(buf, sizeof(buf),
                           "[%s] Speed: %.2f km/h | RPM: %.1f | ⚠️ MISSING MAGNET (corrected)\r\n",
                           method_name,
                           speed_kmh,
                           rpm);
        uart_write_bytes(UART_PORT, buf, len);
    } else {
        // Normal calculation
        float period_s = (float)ticks / (float)TIMER_RESOLUTION_HZ;
        float time_per_rev_s = period_s * MAGNETS_PER_REV;
        float rpm = 60.0f / time_per_rev_s;
        
        float distance_per_pulse_m = wheel_circumference_m / MAGNETS_PER_REV;
        float speed_ms = distance_per_pulse_m / period_s;
        float speed_kmh = speed_ms * 3.6f;

        int len = snprintf(buf, sizeof(buf),
                           "[%s] Speed: %.2f km/h | RPM: %.1f | Period: %.3f ms\r\n",
                           method_name,
                           speed_kmh,
                           rpm,
                           period_s * 1000.0f);
        uart_write_bytes(UART_PORT, buf, len);
    }
}

// ─── Processing task ─────────────────────────────────────────────────────────

static void measurement_task(void *arg)
{
    uint64_t ticks_direct;
    uint64_t ticks_counter;
    
    float wheel_circumference_m = (M_PI * WHEEL_DIAMETER_MM) / 1000.0f;
    
    bool timeout_displayed = false;

    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        TickType_t time_since_last_pulse = current_time - last_pulse_time;
        
        // Check for timeout (no pulse for more than TIMEOUT_MS)
        if (time_since_last_pulse > pdMS_TO_TICKS(TIMEOUT_MS)) {
            if (!timeout_displayed) {
                const char* timeout_msg = "Speed: 0.00 km/h | RPM: 0.0 | Wheel stopped\r\n\r\n";
                uart_write_bytes(UART_PORT, timeout_msg, strlen(timeout_msg));
                timeout_displayed = true;
                
                // Reset history when stopped
                history_count = 0;
                history_index = 0;
            }
        } else {
            timeout_displayed = false;
        }
        
        bool has_direct = ring_pop(&ring_direct, &ticks_direct);
        bool has_counter = ring_pop(&ring_counter, &ticks_counter);
        
        if (has_direct) {
            // Check for anomaly
            float corrected_ticks = 0;
            bool is_anomaly = detect_anomaly(ticks_direct, &corrected_ticks);
            
            // Update history (only with non-anomalous values)
            if (!is_anomaly) {
                update_period_history(ticks_direct);
            }
            
            calculate_and_print(ticks_direct, "DIRECT ", wheel_circumference_m, is_anomaly, corrected_ticks);
        }
        
        if (has_counter) {
            float corrected_ticks = 0;
            bool is_anomaly = detect_anomaly(ticks_counter, &corrected_ticks);
            calculate_and_print(ticks_counter, "COUNTER", wheel_circumference_m, is_anomaly, corrected_ticks);
        }
        
        // Print comparison if we have both measurements
        if (has_direct && has_counter) {
            int64_t diff_ticks = (int64_t)ticks_direct - (int64_t)ticks_counter;
            float diff_ns = (float)diff_ticks * 25.0f;
            
            char comp_buf[128];
            int len = snprintf(comp_buf, sizeof(comp_buf),
                               ">>> Difference: %lld ticks (%.0f ns)\r\n\r\n",
                               diff_ticks,
                               diff_ns);
            uart_write_bytes(UART_PORT, comp_buf, len);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ─── Main ────────────────────────────────────────────────────────────────────

void app_main(void)
{
    init_uart();
    init_timer();
    init_led_strip();
    init_gpio();

    float wheel_circumference_m = (M_PI * WHEEL_DIAMETER_MM) / 1000.0f;
    char banner[600];
    int len = snprintf(banner, sizeof(banner),
                       "\r\n=== Bicycle Speedometer - Dual Method Comparison ===\r\n"
                       "Wheel diameter: %d mm\r\n"
                       "Wheel circumference: %.3f m\r\n"
                       "Magnets per revolution: %d\r\n"
                       "Hall sensor: GPIO %d | LED: GPIO %d\r\n"
                       "Timer resolution: 25 ns per tick\r\n"
                       "Timeout: %d ms (displays 0 km/h if no pulses)\r\n"
                       "Anomaly detection: %.1fx threshold for missing magnets\r\n\r\n"
                       "Method 1 (DIRECT):  Consecutive pulse differences\r\n"
                       "Method 2 (COUNTER): Universal counter (pulse 1->2, calculate, pulse 3 opens new window)\r\n"
                       "======================================================\r\n\r\n",
                       WHEEL_DIAMETER_MM,
                       wheel_circumference_m,
                       MAGNETS_PER_REV,
                       SIGNAL_GPIO,
                       LED_GPIO,
                       TIMEOUT_MS,
                       ANOMALY_THRESHOLD);
    uart_write_bytes(UART_PORT, banner, len);

    xTaskCreate(led_task, "led", 2048, NULL, 10, NULL);
    xTaskCreate(measurement_task, "measurement", 4096, NULL, 5, NULL);
    
    vTaskDelete(NULL);
}