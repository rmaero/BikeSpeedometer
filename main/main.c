#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// ─── Config ──────────────────────────────────────────────────────────────────

#define SIGNAL_GPIO         GPIO_NUM_4
#define TIMER_RESOLUTION_HZ 40000000    // 40 MHz → 25 ns per tick (max for gptimer)

#define UART_PORT           UART_NUM_0
#define UART_BAUD           115200
#define UART_TX_PIN         GPIO_NUM_1
#define UART_RX_PIN         GPIO_NUM_3
#define UART_BUF_SIZE       256

#define RING_BUF_SIZE       16
#define RING_BUF_MASK       (RING_BUF_SIZE - 1)

// ─── Ring buffer ─────────────────────────────────────────────────────────────

typedef struct {
    uint64_t data[RING_BUF_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} ring_buf_t;

static ring_buf_t ring = {0};

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
static volatile uint64_t last_timestamp = 0;
static volatile bool     first_edge     = true;

// ─── ISR ─────────────────────────────────────────────────────────────────────

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint64_t now = 0;
    gptimer_get_raw_count(gptimer, &now);

    if (first_edge) {
        first_edge = false;
    } else {
        ring_push(&ring, now - last_timestamp);
    }

    last_timestamp = now;
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
        .clk_src       = GPTIMER_CLK_SRC_APB,  // 80 MHz base clock
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,  // Divide by 2 → 40 MHz
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

// ─── GPIO init ───────────────────────────────────────────────────────────────

static void init_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SIGNAL_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SIGNAL_GPIO, gpio_isr_handler, NULL));
}

// ─── Main loop ───────────────────────────────────────────────────────────────

void app_main(void)
{
    init_uart();
    init_timer();
    init_gpio();

    const char *banner = "Pulse period measurement ready (25 ns resolution).\r\n";
    uart_write_bytes(UART_PORT, banner, strlen(banner));

    char buf[80];
    uint64_t ticks;

    while (1) {
        if (ring_pop(&ring, &ticks)) {
            // Convert ticks to nanoseconds
            // ticks * (1e9 / TIMER_RESOLUTION_HZ) = ticks * 25
            uint64_t period_ns   = ticks * 25;
            uint32_t period_us   = (uint32_t)(period_ns / 1000);
            uint32_t period_ns_frac = (uint32_t)(period_ns % 1000);

            // Frequency calculation
            uint32_t freq_hz  = (period_ns > 0) ? (uint32_t)(1000000000ULL / period_ns) : 0;
            uint32_t freq_mhz = (period_ns > 0) ? (uint32_t)((1000000000000ULL / period_ns) % 1000) : 0;

            int len = snprintf(buf, sizeof(buf),
                               "Period: %lu.%03lu us | Freq: %lu.%03lu Hz\r\n",
                               (unsigned long)period_us,
                               (unsigned long)period_ns_frac,
                               (unsigned long)freq_hz,
                               (unsigned long)freq_mhz);

            uart_write_bytes(UART_PORT, buf, len);
        }
        
        // Yield to FreeRTOS IDLE task to feed the watchdog
        vTaskDelay(1);  // Delay for 1 tick (~10ms) - won't affect measurement accuracy
    }
}