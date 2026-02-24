#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>



// ─── Config ──────────────────────────────────────────────────────────────────

#define SIGNAL_GPIO         GPIO_NUM_27  // Hall sensor input

#define LED_R GPIO_NUM_4
#define LED_G GPIO_NUM_16
#define LED_B GPIO_NUM_17

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
#define WHEEL_DIAMETER_MM   750          // Wheel diameter in millimeters

// CYD LCD Configuration (ILI9341)
#define LCD_HOST            SPI2_HOST
#define LCD_V_RES           320
#define LCD_H_RES           240
#define LCD_PIXEL_CLK_HZ    (40 * 1000 * 1000)
#define LCD_BK_LIGHT_ON     1
#define LCD_BK_LIGHT_OFF    0

#define PIN_LCD_MOSI        GPIO_NUM_13
#define PIN_LCD_CLK         GPIO_NUM_14
#define PIN_LCD_CS          GPIO_NUM_15
#define PIN_LCD_DC          GPIO_NUM_2
#define PIN_LCD_RST         GPIO_NUM_12
#define PIN_LCD_BL          GPIO_NUM_21

// Display smoothing
#define DISPLAY_AVG_SIZE    4            // Average last 4 measurements for display

// ─── Ring buffers ────────────────────────────────────────────────────────────

typedef struct {
    uint64_t data[RING_BUF_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} ring_buf_t;

static ring_buf_t ring_direct = {0};
static ring_buf_t ring_counter = {0};

static inline bool IRAM_ATTR ring_push(ring_buf_t *rb, uint64_t value) //guarda valor nuevo en el buffer
{
    uint32_t next = (rb->write_idx + 1) & RING_BUF_MASK;
    if (next == rb->read_idx) {
        return false;
    }
    rb->data[rb->write_idx] = value;
    rb->write_idx = next;
    return true;
}

static inline bool ring_pop(ring_buf_t *rb, uint64_t *out) //toma el ultimo valor nuevo del buffer
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

//manejo LCD
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
// LVGL UI elements
static lv_obj_t *speed_label = NULL;
static lv_obj_t *rpm_label = NULL;
static lv_obj_t *status_label = NULL;
static lv_obj_t *method_label = NULL;

// Contador Continuo
static volatile uint64_t last_timestamp_direct = 0;
static volatile bool     first_edge_direct     = true;

// CUR
static volatile uint64_t window_start = 0;
static volatile uint64_t calculated_period = 0;
static volatile uint8_t  pulse_count = 0;
static volatile bool     first_edge_counter = true;

//bandera para led testigo del pulso del sensor
static volatile bool     trigger_led    = false;

// Timeout para rueda detenida
static volatile TickType_t last_pulse_time = 0;

// array para deteccion de iman perdido
// guardo hasta los ultimo 8 valores y los promedio para comparar contra el nuevo periodo
#define HISTORY_SIZE 8
static uint64_t period_history[HISTORY_SIZE] = {0};
static uint8_t history_index = 0;
static uint8_t history_count = 0;

// Arrays para promediar valores del display
static float speed_history[DISPLAY_AVG_SIZE] = {0};
static float rpm_history[DISPLAY_AVG_SIZE] = {0};
static float period_history_display[DISPLAY_AVG_SIZE] = {0};
static uint8_t display_index = 0;
static uint8_t display_count = 0;

// ─── ISR ─────────────────────────────────────────────────────────────────────
//interrupcion
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint64_t now = 0;
    gptimer_get_raw_count(gptimer, &now); //guardo valor del timer

    //guardo el tiempo en el que se da el ultimo pulso para mas adelante chequear rueda detenida
    last_pulse_time = xTaskGetTickCountFromISR(); 

    // ContadorContinuo  //TIEMPO ENTRE PULSOS CONSECUTIVOS
    if (first_edge_direct) { //si es el primer pulso que llega no guardar en el ringbuffer
        first_edge_direct = false;
    } else {
        ring_push(&ring_direct, now - last_timestamp_direct); //TIEMPO ACTUAL - ANTERIOR
    }
    last_timestamp_direct = now; //guardo el valor actual como ultimo valor, para el pulso siguiente

    // Method 2: Universal counter (alternating windows)
    if (first_edge_counter) { //abro la ventana de cuenta por primera vez
        window_start = now;
        pulse_count = 1;
        first_edge_counter = false;
    } else {
        pulse_count++;
        
        if (pulse_count == 2) { //cuando llega el segundo pulso se cierra la ventana de cuenta y guardo el valor en el buffer
            calculated_period = now - window_start;
            ring_push(&ring_counter, calculated_period);
        } else if (pulse_count == 3) { //en el tercer pulso vuelvo a abrir la ventana de cuenta y se reinicia la secuencia
            window_start = now;
            pulse_count = 1; //tercer pulso es igual al primer pulso
        }
    }

    trigger_led = true; //bandera para prender el led
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

// ─── LCD init ────────────────────────────────────────────────────────────────

static void init_lcd(void)
{
    // Configure backlight
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = (1ULL << PIN_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(PIN_LCD_BL, LCD_BK_LIGHT_ON);

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Configure LCD panel IO
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &lcd_io_handle));

    // Configure LCD panel - use the generic st7789 which works with ILI9341
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    
    // Try to create ILI9341 panel using st7789 driver (they're compatible)
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &lcd_panel));
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, false, false));
    // Set the display area to 320x240 (ILI9341 size)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));
}

// ─── LVGL init ───────────────────────────────────────────────────────────────

static void init_lvgl_ui(void)
{
    // Initialize LVGL port
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // Add LCD display
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io_handle,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_V_RES * 50, //LCD_H_RES EN VERTICAL
        .double_buffer = true,
        .hres = LCD_V_RES, //INVIERTO PARA HORIZONTAL
        .vres = LCD_H_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = true, //f vertical
            .mirror_x = false,//f vertical// antes true
            .mirror_y = true,//f vertical
        }
    };
    lvgl_port_add_disp(&disp_cfg);

    // Lock LVGL for UI creation
    lvgl_port_lock(0);

    // Set background color
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "TP3 - MEDICIONES");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Speed label (large)
    speed_label = lv_label_create(scr);
    lv_label_set_text(speed_label, "0.0 km/h");
    lv_obj_set_style_text_color(speed_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
    lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -60);

    // RPM label
    rpm_label = lv_label_create(scr);
    lv_label_set_text(rpm_label, "RPM: 0.0");
    lv_obj_set_style_text_color(rpm_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(rpm_label, &lv_font_montserrat_48, 0);
    lv_obj_align(rpm_label, LV_ALIGN_CENTER, 0, 10);

    // Status label
    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Ready");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_20, 0);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -50);

    // Method label
    method_label = lv_label_create(scr);
    lv_label_set_text(method_label, "PERIODO:");// ESTO NO ES LO QUE IMPRIME?
    lv_obj_set_style_text_color(method_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(method_label, &lv_font_montserrat_20, 0);
    lv_obj_align(method_label, LV_ALIGN_BOTTOM_MID, 0, -10);

    lvgl_port_unlock();
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

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_R) |
                        (1ULL << LED_G) |
                        (1ULL << LED_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SIGNAL_GPIO, gpio_isr_handler, NULL));

    // Apagar todo (active LOW -> 1 apaga)
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 1);
}

// ─── LED control task ────────────────────────────────────────────────────────
// este task esta para generar un delay de 20ms despues de la ISR y luego apagar el LED
static void led_task(void *arg)
{
    TickType_t led_off_time = 0;
    bool led_is_on = false;

    while (1) {
        if (trigger_led) {
            trigger_led = false;
            
            gpio_set_level(LED_G, 0); //prendo led
            
            led_is_on = true;
            led_off_time = xTaskGetTickCount() + pdMS_TO_TICKS(LED_PULSE_MS);
        }

        if (led_is_on && (xTaskGetTickCount() >= led_off_time)) {
            gpio_set_level(LED_G, 1); //apago led
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
        return false;
    }
    
    float avg = get_average_period();
    float ratio = (float)current_period / avg;
    
    if (ratio > ANOMALY_THRESHOLD) {
        *corrected_period = avg;
        return true;
    }
    
    return false;
}

// ─── Display averaging functions ─────────────────────────────────────────────

static void update_display_history(float speed, float rpm, float period)
{
    speed_history[display_index] = speed;
    rpm_history[display_index] = rpm;
    period_history_display[display_index] = period;
    
    display_index = (display_index + 1) % DISPLAY_AVG_SIZE;
    if (display_count < DISPLAY_AVG_SIZE) {
        display_count++;
    }
}

static float get_average_speed(void)
{
    if (display_count == 0) return 0.0f;
    
    float sum = 0;
    for (uint8_t i = 0; i < display_count; i++) {
        sum += speed_history[i];
    }
    return sum / (float)display_count;
}

static float get_average_rpm(void)
{
    if (display_count == 0) return 0.0f;
    
    float sum = 0;
    for (uint8_t i = 0; i < display_count; i++) {
        sum += rpm_history[i];
    }
    return sum / (float)display_count;
}

static float get_average_period_display(void)
{
    if (display_count == 0) return 0.0f;
    
    float sum = 0;
    for (uint8_t i = 0; i < display_count; i++) {
        sum += period_history_display[i];
    }
    return sum / (float)display_count;
}

static void reset_display_history(void)
{
    display_count = 0;
    display_index = 0;
    for (uint8_t i = 0; i < DISPLAY_AVG_SIZE; i++) {
        speed_history[i] = 0.0f;
        rpm_history[i] = 0.0f;
        period_history_display[i] = 0.0f;
    }
}

// ─── Update LCD display ──────────────────────────────────────────────────────

static void update_lcd_display(float speed_kmh, float rpm, float period, const char* status)
{
    lvgl_port_lock(0);
    
    char speed_text[32];
    snprintf(speed_text, sizeof(speed_text), "%.1f km/h", speed_kmh);
    lv_label_set_text(speed_label, speed_text);
    
    char rpm_text[32];
    snprintf(rpm_text, sizeof(rpm_text), "RPM: %.1f", rpm);
    lv_label_set_text(rpm_label, rpm_text);
    
    lv_label_set_text(status_label, status);
    
    char method_text[64];
    snprintf(method_text, sizeof(method_text), "PERIODO: %.3f s", period);
    lv_label_set_text(method_label, method_text);
    
    lvgl_port_unlock();
}

// ─── Processing task ─────────────────────────────────────────────────────────
// funcion principal de calculo y display
static void measurement_task(void *arg)
{
    uint64_t ticks_direct;
    uint64_t ticks_counter;
    
    float wheel_circumference_m = (M_PI * WHEEL_DIAMETER_MM) / 1000.0f;
    
    bool timeout_displayed = false;

    while (1) {
        TickType_t current_time = xTaskGetTickCount();                          //tomo el tiempo del sistema
        TickType_t time_since_last_pulse = current_time - last_pulse_time;      //calcula cuanto paso desde el ultimo pulso recibido para rueda detenida
        
        // Check for timeout
        //si paso mas tiempo del maximo permitido (aprox 2.2 seg) mostrar rueda detenida
        if (time_since_last_pulse > pdMS_TO_TICKS(TIMEOUT_MS)) {
            if (!timeout_displayed) {
                //mostrar 0 kmh en display y por UART
                const char* timeout_msg = "Speed: 0.00 km/h | RPM: 0.0 | Wheel stopped\r\n\r\n";
                uart_write_bytes(UART_PORT, timeout_msg, strlen(timeout_msg));
                
                // Reset display history and show zeros
                reset_display_history();
                update_lcd_display(0.0f, 0.0f, 0.0f, "RUEDA DETENIDA");
                
                //uso esta bandera para mostrar una unica vez el mensaje
                timeout_displayed = true;
                history_count = 0;
                history_index = 0;
            }
        } else {
            timeout_displayed = false;
        }
        

        //tomo los valores de ticks de los ring buffer de ContadorContinuo y CUR respectivamente
        bool has_direct = ring_pop(&ring_direct, &ticks_direct);
        bool has_counter = ring_pop(&ring_counter, &ticks_counter);
        
        if (has_direct) {
            float corrected_ticks = 0;
            bool is_anomaly = detect_anomaly(ticks_direct, &corrected_ticks);
            
            if (!is_anomaly) {
                update_period_history(ticks_direct);
            }
            
            // Calculate speed and RPM
            float period_s = (is_anomaly ? corrected_ticks : (float)ticks_direct) / (float)TIMER_RESOLUTION_HZ;
            float time_per_rev_s = period_s * MAGNETS_PER_REV;
            float rpm = 60.0f / time_per_rev_s;
            
            float distance_per_pulse_m = wheel_circumference_m / MAGNETS_PER_REV;
            float speed_ms = distance_per_pulse_m / period_s;
            float speed_kmh = speed_ms * 3.6f;
            
            // Update display history with new values
            update_display_history(speed_kmh, rpm, period_s);
            
            // Get averaged values for display
            float avg_speed = get_average_speed();
            float avg_rpm = get_average_rpm();
            float avg_period = get_average_period_display();
            
            // Update LCD with averaged values
            const char* status = is_anomaly ? "MISSING MAGNET" : "Running";
            update_lcd_display(avg_speed, avg_rpm, avg_period, status);
            
            // UART output (still shows instantaneous values)
            char buf[200];
            if (is_anomaly) {
                int len = snprintf(buf, sizeof(buf),
                                   "[DIRECT ] Speed: %.2f km/h | RPM: %.1f | ⚠️ MISSING MAGNET (corrected)\r\n",
                                   speed_kmh, rpm);
                uart_write_bytes(UART_PORT, buf, len);
            } else {
                int len = snprintf(buf, sizeof(buf),
                                   "[DIRECT ] Speed: %.2f km/h | RPM: %.1f | Period: %.3f ms\r\n",
                                   speed_kmh, rpm, period_s * 1000.0f);
                uart_write_bytes(UART_PORT, buf, len);
            }
        }
        
        if (has_counter) {
            float corrected_ticks = 0;
            bool is_anomaly = detect_anomaly(ticks_counter, &corrected_ticks);
            
            char buf[200];
            float period_s = (is_anomaly ? corrected_ticks : (float)ticks_counter) / (float)TIMER_RESOLUTION_HZ;
            float time_per_rev_s = period_s * MAGNETS_PER_REV;
            float rpm = 60.0f / time_per_rev_s;
            
            float distance_per_pulse_m = wheel_circumference_m / MAGNETS_PER_REV;
            float speed_ms = distance_per_pulse_m / period_s;
            float speed_kmh = speed_ms * 3.6f;
            
            if (is_anomaly) {
                int len = snprintf(buf, sizeof(buf),
                                   "[COUNTER] Speed: %.2f km/h | RPM: %.1f | ⚠️ MISSING MAGNET (corrected)\r\n",
                                   speed_kmh, rpm);
                uart_write_bytes(UART_PORT, buf, len);
            } else {
                int len = snprintf(buf, sizeof(buf),
                                   "[COUNTER] Speed: %.2f km/h | RPM: %.1f | Period: %.3f ms\r\n",
                                   speed_kmh, rpm, period_s * 1000.0f);
                uart_write_bytes(UART_PORT, buf, len);
            }
        }
        
        if (has_direct && has_counter) {
            int64_t diff_ticks = (int64_t)ticks_direct - (int64_t)ticks_counter;
            float diff_ns = (float)diff_ticks * 25.0f;
            
            char comp_buf[128];
            int len = snprintf(comp_buf, sizeof(comp_buf),
                               ">>> Difference: %lld ticks (%.0f ns)\r\n\r\n",
                               diff_ticks, diff_ns);
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
    init_lcd();
    init_lvgl_ui();
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
                       "Anomaly detection: %.1fx threshold for missing magnets\r\n"
                       "Display averaging: last %d measurements\r\n"
                       "LCD: %dx%d ILI9341\r\n\r\n"
                       "Method 1 (DIRECT):  Consecutive pulse differences\r\n"
                       "Method 2 (COUNTER): Universal counter\r\n"
                       "======================================================\r\n\r\n",
                       WHEEL_DIAMETER_MM,
                       wheel_circumference_m,
                       MAGNETS_PER_REV,
                       SIGNAL_GPIO,
                       LED_G,
                       TIMEOUT_MS,
                       ANOMALY_THRESHOLD,
                       DISPLAY_AVG_SIZE,
                       LCD_H_RES, LCD_V_RES);
    uart_write_bytes(UART_PORT, banner, len);

    xTaskCreate(led_task, "led", 2048, NULL, 10, NULL);
    xTaskCreate(measurement_task, "measurement", 8192, NULL, 5, NULL);
    
    vTaskDelete(NULL);
}