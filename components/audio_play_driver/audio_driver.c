#include "audio_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/i2c.h"

#include "led_strip.h"
#include "bsp_board.h"

// --------------------------
// I2C config
// --------------------------
#define ES_I2C_PORT        0
#define ES_I2C_SCL_PIN     10
#define ES_I2C_SDA_PIN     11
#define ES_I2C_FREQ_HZ     100000

// ES8311 DAC
#define ES8311_ADDR        0x18


// LED strip handle
static led_strip_handle_t s_led_strip = NULL;

void led_ring_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_strip) return;
    for (int i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        led_strip_set_pixel(s_led_strip, i, r, g, b);
    }
    led_strip_refresh(s_led_strip);
}

void led_ring_clear(void)
{
    if (!s_led_strip) return;
    led_strip_clear(s_led_strip);
    led_strip_refresh(s_led_strip);
}
// ES7210 ADC
#define ES7210_ADDR        0x40   // seen in I2C scan

// --------------------------
// I2S pins (from schematic)
// --------------------------
#define I2S_MCLK_PIN   12
#define I2S_BCLK_PIN   13
#define I2S_LRCLK_PIN  14
#define I2S_DIN_PIN    15    // ES7210 SDOUT -> ESP32 (mic)
#define I2S_DOUT_PIN   16    // ESP32 -> ES8311 (speaker)

// --------------------------
// ES7210 register map
// --------------------------
#define ES7210_RESET_REG00           0x00 /* Reset control */
#define ES7210_CLOCK_OFF_REG01       0x01 /* Used to turn off the ADC clock */
#define ES7210_MAINCLK_REG02         0x02 /* Set ADC clock frequency division */
#define ES7210_MASTER_CLK_REG03      0x03 /* MCLK source & SCLK division */
#define ES7210_LRCK_DIVH_REG04       0x04 /* lrck_divh */
#define ES7210_LRCK_DIVL_REG05       0x05 /* lrck_divl */
#define ES7210_POWER_DOWN_REG06      0x06 /* power down */
#define ES7210_OSR_REG07             0x07
#define ES7210_MODE_CONFIG_REG08     0x08 /* Set master/slave & channels */
#define ES7210_TIME_CONTROL0_REG09   0x09 /* init state period */
#define ES7210_TIME_CONTROL1_REG0A   0x0A /* power up period */
#define ES7210_SDP_INTERFACE1_REG11  0x11 /* sample bits & fmt */
#define ES7210_SDP_INTERFACE2_REG12  0x12 /* pins / TDM */
#define ES7210_ADC_AUTOMUTE_REG13    0x13 /* mute */
#define ES7210_ADC34_MUTERANGE_REG14 0x14 /* mute range */

#define ES7210_ANALOG_REG40          0x40 /* ANALOG Power */
#define ES7210_MIC12_BIAS_REG41      0x41
#define ES7210_MIC34_BIAS_REG42      0x42
#define ES7210_MIC1_GAIN_REG43       0x43
#define ES7210_MIC2_GAIN_REG44       0x44
#define ES7210_MIC3_GAIN_REG45       0x45
#define ES7210_MIC4_GAIN_REG46       0x46
#define ES7210_MIC1_POWER_REG47      0x47
#define ES7210_MIC2_POWER_REG48      0x48
#define ES7210_MIC3_POWER_REG49      0x49
#define ES7210_MIC4_POWER_REG4A      0x4A
#define ES7210_MIC12_POWER_REG4B     0x4B /* MICBias & ADC & PGA Power */
#define ES7210_MIC34_POWER_REG4C     0x4C

// --------------------------
// Globals
// --------------------------
static const char *TAG = "audio_driver";

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t tca9555_dev = NULL;
static i2c_master_dev_handle_t es8311_dev = NULL;
static i2c_master_dev_handle_t es7210_dev = NULL;

static i2s_chan_handle_t i2s_tx_handle = NULL;   // speaker
static i2s_chan_handle_t i2s_rx_handle = NULL;   // mic

// Whether codec is in TDM mode (4 slots per LRCK)
static bool s_es7210_tdm = false;

bool audio_driver_es7210_is_tdm(void)
{
    return s_es7210_tdm;
}

// --------------------------
// I2C helpers
// --------------------------
static void i2c_scan_bus(i2c_master_bus_handle_t bus)
{
    ESP_LOGI(TAG, "Starting I2C scan on this bus...");
    for (int addr = 0x03; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            ESP_LOGW(TAG, "I2C device found at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan complete");
}

// ES8311
static esp_err_t es8311_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    return i2c_master_transmit(es8311_dev, data, sizeof(data), 100);
}

static esp_err_t es8311_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit(es8311_dev, &reg, 1, 100);
    if (err != ESP_OK) return err;
    return i2c_master_receive(es8311_dev, val, 1, 100);
}

// ES7210
static esp_err_t es7210_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    return i2c_master_transmit(es7210_dev, data, sizeof(data), 100);
}

static esp_err_t es7210_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit(es7210_dev, &reg, 1, 100);
    if (err != ESP_OK) return err;
    return i2c_master_receive(es7210_dev, val, 1, 100);
}

static esp_err_t es7210_update_reg_bit(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur = 0;
    esp_err_t err = es7210_read_reg(reg, &cur);
    if (err != ESP_OK) return err;
    cur &= ~mask;
    cur |= (val & mask);
    return es7210_write_reg(reg, cur);
}

// --------------------------
// ES8311 DAC init (speaker)
// --------------------------
static esp_err_t es8311_init_codec(void)
{
    ESP_LOGI(TAG, "Initializing ES8311 DAC...");

    // Reset
    ESP_ERROR_CHECK(es8311_write_reg(0x00, 0x3F));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(es8311_write_reg(0x00, 0x00));

    // Power & clock
    ESP_ERROR_CHECK(es8311_write_reg(0x01, 0x3F));   // all power on
    ESP_ERROR_CHECK(es8311_write_reg(0x02, 0x00));   // ADC power on (safe)
    ESP_ERROR_CHECK(es8311_write_reg(0x03, 0x10));   // I2S slave

    // I2S format: 16‑bit, I2S standard
    ESP_ERROR_CHECK(es8311_write_reg(0x10, 0x00));   // ADC I2S (unused)
    ESP_ERROR_CHECK(es8311_write_reg(0x11, 0x00));   // DAC I2S

    // Unmute ADC/DAC
    ESP_ERROR_CHECK(es8311_write_reg(0x14, 0x00));
    ESP_ERROR_CHECK(es8311_write_reg(0x15, 0x00));

    // MICBIAS (not really used here, but harmless)
    ESP_ERROR_CHECK(es8311_write_reg(0x1A, 0x0C));

    // DAC path
    ESP_ERROR_CHECK(es8311_write_reg(0x25, 0x00));   // DAC unmute
    ESP_ERROR_CHECK(es8311_write_reg(0x26, 0x0C));   // DAC volume mid

    // Output driver (board dependent; conservative defaults)
    ESP_ERROR_CHECK(es8311_write_reg(0x31, 0x00));
    ESP_ERROR_CHECK(es8311_write_reg(0x32, 0x00));
    ESP_ERROR_CHECK(es8311_write_reg(0x33, 0x10));

    // Final unmute
    ESP_ERROR_CHECK(es8311_write_reg(0x36, 0x00));
    ESP_ERROR_CHECK(es8311_write_reg(0x37, 0x00));

    // Instrumentation
    uint8_t reg;
    es8311_read_reg(0x02, &reg);
    ESP_LOGI(TAG, "ES8311 REG02 (ADC power) = 0x%02X", reg);
    es8311_read_reg(0x1A, &reg);
    ESP_LOGI(TAG, "ES8311 REG1A (MICBIAS) = 0x%02X", reg);
    es8311_read_reg(0x23, &reg);
    ESP_LOGI(TAG, "ES8311 REG23 (PGA gain) = 0x%02X", reg);

    ESP_LOGI(TAG, "ES8311 initialized");
    return ESP_OK;
}
// --------------------------

static esp_err_t es7210_init_adc(void)
{
    // Enable MIC1 & MIC2 with gain, mirroring es7210_mic_select/_set_channel_gain behavior
    // Enable MIC1
    ESP_LOGI(TAG, "Enable ES7210_INPUT_MIC1");
    // Clear clock-off bits for MIC1/2 (0x0B mask)
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_CLOCK_OFF_REG01, 0x0B, 0x00));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC12_POWER_REG4B, 0x00));   // power MIC1/2 path
    // MIC1 gain enable bit (0x10) and gain value (0x0F mask)
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MIC1_GAIN_REG43, 0x10, 0x10)); // enable gain
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MIC1_GAIN_REG43, 0x0F, 0x0C)); // ~12 dB

    // Enable MIC2
    ESP_LOGI(TAG, "Enable ES7210_INPUT_MIC2");
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_CLOCK_OFF_REG01, 0x0B, 0x00));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC12_POWER_REG4B, 0x00));   // ensure MIC1/2 path on
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MIC2_GAIN_REG44, 0x10, 0x10)); // enable gain
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MIC2_GAIN_REG44, 0x0F, 0x0C)); // ~12 dB

    // Optionally leave MIC3/4 off for now

    // Unmute / automute off if needed
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_ADC_AUTOMUTE_REG13, 0x00));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_ADC34_MUTERANGE_REG14, 0x00));

    // Final reset tick as per es7210_start()
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_RESET_REG00, 0x71));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_RESET_REG00, 0x41));

    
    uint8_t r1, r2, r40;
    es7210_read_reg(ES7210_CLOCK_OFF_REG01, &r1);
    es7210_read_reg(ES7210_POWER_DOWN_REG06, &r2);
    es7210_read_reg(ES7210_ANALOG_REG40, &r40);
    ESP_LOGI(TAG, "ES7210 CLOCK_OFF=0x%02X PWDN=0x%02X ANALOG40=0x%02X", r1, r2, r40);

    uint8_t mic1g, mic2g;
    es7210_read_reg(ES7210_MIC1_GAIN_REG43, &mic1g);
    es7210_read_reg(ES7210_MIC2_GAIN_REG44, &mic2g);
    ESP_LOGI(TAG, "ES7210 MIC1_GAIN=0x%02X MIC2_GAIN=0x%02X", mic1g, mic2g);

    ESP_LOGI(TAG, "ES7210 ADC initialized");
    return ESP_OK;
}

// --------------------------
// TCA9555 GPIO expander init (uses i2c_master shim)
// --------------------------
#define TCA9555_ADDR  0x20
void tca9555_init(void) {
    // Configure both Port0 and Port1 as inputs (configuration regs 0x06 and 0x07)
    uint8_t config_val = 0xFF;  // All bits input

    if (!tca9555_dev) {
        i2c_device_config_t cfg = { .device_address = TCA9555_ADDR, .scl_speed_hz = ES_I2C_FREQ_HZ };
        if (i2c_master_bus_add_device(i2c_bus, &cfg, &tca9555_dev) != ESP_OK) return;
    }

    uint8_t cfg0[2] = { 0x06, config_val };
    uint8_t cfg1[2] = { 0x07, config_val };
    i2c_master_transmit(tca9555_dev, cfg0, sizeof(cfg0), 1000 / portTICK_PERIOD_MS);
    i2c_master_transmit(tca9555_dev, cfg1, sizeof(cfg1), 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "tca9555_init: configured ports 0 and 1 as inputs (0x%02X)", config_val);

    // Dump registers for diagnosis with per-register logging and warnings
    {
        uint8_t regs[8];
        bool ok_all = true;
        for (uint8_t r = 0; r < 8; ++r) {
            uint8_t reg = r;
            regs[r] = 0xEE;
            esp_err_t err = i2c_master_transmit(tca9555_dev, &reg, 1, 1000 / portTICK_PERIOD_MS);
            if (err != ESP_OK) {
                regs[r] = 0xEF;
                ok_all = false;
                ESP_LOGW(TAG, "tca9555: failed to write reg addr 0x%02X (%s)", reg, esp_err_to_name(err));
                continue;
            }
            err = i2c_master_receive(tca9555_dev, &regs[r], 1, 1000 / portTICK_PERIOD_MS);
            if (err != ESP_OK) {
                regs[r] = 0xEE;
                ok_all = false;
                ESP_LOGW(TAG, "tca9555: failed to read reg 0x%02X (%s)", reg, esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "tca9555: reg 0x%02X = 0x%02X", reg, regs[r]);
            }
        }
        ESP_LOGI(TAG, "tca9555_regs 0x00..0x07: %02X %02X %02X %02X %02X %02X %02X %02X",
                 regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
        if (!ok_all) {
            ESP_LOGW(TAG, "tca9555: some register reads failed; verify wiring/address/pull-ups");
        }
    }
}

// Read Port 1
uint8_t tca9555_read_port1(void) {
    uint8_t reg = 0x01;
    uint8_t value = 0;

    if (!tca9555_dev) {
        i2c_device_config_t cfg = { .device_address = TCA9555_ADDR, .scl_speed_hz = ES_I2C_FREQ_HZ };
        if (i2c_master_bus_add_device(i2c_bus, &cfg, &tca9555_dev) != ESP_OK) return 0xFF;
    }

    if (i2c_master_transmit(tca9555_dev, &reg, 1, 1000 / portTICK_PERIOD_MS) != ESP_OK) return 0xFF;
    if (i2c_master_receive(tca9555_dev, &value, 1, 1000 / portTICK_PERIOD_MS) != ESP_OK) return 0xFF;

    return value;
}

// Read Port 0
uint8_t tca9555_read_port0(void) {
    uint8_t reg = 0x00;
    uint8_t value = 0;

    if (!tca9555_dev) {
        i2c_device_config_t cfg = { .device_address = TCA9555_ADDR, .scl_speed_hz = ES_I2C_FREQ_HZ };
        if (i2c_master_bus_add_device(i2c_bus, &cfg, &tca9555_dev) != ESP_OK) return 0xFF;
    }

    if (i2c_master_transmit(tca9555_dev, &reg, 1, 1000 / portTICK_PERIOD_MS) != ESP_OK) return 0xFF;
    if (i2c_master_receive(tca9555_dev, &value, 1, 1000 / portTICK_PERIOD_MS) != ESP_OK) return 0xFF;

    return value;
}

// Low-level fallback read using ESP-IDF i2c driver in case the master wrapper
// fails to read the expander registers correctly.
static esp_err_t tca9555_raw_read_reg(uint8_t reg, uint8_t *out)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = ESP_ERR_INVALID_ARG;
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    // Repeated start and read one byte
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9555_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, out, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(ES_I2C_PORT, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    return err;
}

bool key3_pressed(void) {
    static uint8_t prev_p0 = 0xFF;
    static uint8_t prev_p1 = 0xFF;
    static bool prev_pressed = false;

    uint8_t port0 = tca9555_read_port0();
    uint8_t port1 = tca9555_read_port1();
    bool used_raw = false;

    // If both ports read as all-zero (or invalid sentinel), try low-level read
    // which may succeed where the wrapper doesn't.
    if ((port0 == 0x00 && port1 == 0x00) || (port0 == 0xFF || port1 == 0xFF)) {
        uint8_t raw0 = 0xEE, raw1 = 0xEE;
        if (tca9555_raw_read_reg(0x00, &raw0) == ESP_OK) port0 = raw0;
        if (tca9555_raw_read_reg(0x01, &raw1) == ESP_OK) port1 = raw1;
        used_raw = true;
    }

    bool pressed_p1 = !(port1 & (1 << 1));  // P1_1, active-low expected
    bool pressed_p0 = !(port0 & (1 << 1));  // fallback: P0_1
    bool pressed = pressed_p1 || pressed_p0;

    // Only log when values or pressed state change to avoid flooding the serial
    if (port0 != prev_p0 || port1 != prev_p1) {
        if (used_raw) {
            ESP_LOGI(TAG, "tca9555_raw_read: P0=0x%02X P1=0x%02X", port0, port1);
        } else {
            ESP_LOGD(TAG, "tca9555_read: P0=0x%02X P1=0x%02X", port0, port1);
        }
        prev_p0 = port0;
        prev_p1 = port1;
    }

    if (pressed != prev_pressed) {
        ESP_LOGI(TAG, "key3 state: pressed=%d (P0=0x%02X P1=0x%02X)", (int)pressed, port0, port1);
        prev_pressed = pressed;
    }

    return pressed;
}

bool debounce_key3(void) {
    static uint32_t last_change = 0;
    static bool last_state = false;

    bool current = key3_pressed();
    uint32_t now = xTaskGetTickCount();

    // On first call after boot, initialize the debounce state to the
    // current reading instead of treating it as a press edge. This
    // avoids false positives at startup caused by uninitialized static
    // values and the tick count being greater than the debounce window.
    if (last_change == 0) {
        last_state = current;
        last_change = now;
        ESP_LOGI(TAG, "debounce_key3: initial state=%d (tick %u)", (int)last_state, (unsigned)now);
        return last_state;
    }

    if (current != last_state) {
        // state changed; if stable for debounce interval, accept it
        if ((now - last_change) > pdMS_TO_TICKS(100)) {
            last_state = current;
            last_change = now;
            ESP_LOGI(TAG, "debounce_key3: state change -> %d (port read at tick %u)", (int)last_state, (unsigned)now);
        } else {
            // record change time but do not yet accept
            last_change = now;
            ESP_LOGD(TAG, "debounce_key3: transient change, waiting (now=%u)", (unsigned)now);
        }
    }

    return last_state;
}

// --------------------------
// Public init
// --------------------------
esp_err_t audio_driver_init(i2s_chan_handle_t *tx_handle_out,
                            i2s_chan_handle_t *rx_handle_out)
{
    //
    // --- I2C BUS + DEVICES INIT ---
    //
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = ES_I2C_PORT,
        .sda_io_num = ES_I2C_SDA_PIN,
        .scl_io_num = ES_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { .enable_internal_pullup = 1 },
    };

    ESP_LOGI(TAG, "Creating I2C bus...");
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // Ensure the underlying ESP-IDF I2C driver is installed so that
    // raw `i2c_master_cmd_begin` calls (used by our fallback) succeed.
    {
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = ES_I2C_SDA_PIN,
            .scl_io_num = ES_I2C_SCL_PIN,
            .sda_pullup_en = true,
            .scl_pullup_en = true,
            .master = { .clk_speed = ES_I2C_FREQ_HZ },
        };
        i2c_param_config(ES_I2C_PORT, &i2c_conf);
        esp_err_t rc = i2c_driver_install(ES_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
        if (rc == ESP_OK) {
            ESP_LOGI(TAG, "ESP-IDF i2c driver installed on port %d", ES_I2C_PORT);
        } else if (rc == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "ESP-IDF i2c driver already installed on port %d", ES_I2C_PORT);
        } else {
            ESP_LOGW(TAG, "Failed to install i2c driver: %s", esp_err_to_name(rc));
        }
    }

    i2c_device_config_t es8311_cfg = {
        .device_address = ES8311_ADDR,
        .scl_speed_hz = ES_I2C_FREQ_HZ,
    };
    i2c_device_config_t es7210_cfg = {
        .device_address = ES7210_ADDR,
        .scl_speed_hz = ES_I2C_FREQ_HZ,
    };

    ESP_LOGI(TAG, "Adding ES8311 device...");
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &es8311_cfg, &es8311_dev));

    ESP_LOGI(TAG, "Adding ES7210 device...");
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &es7210_cfg, &es7210_dev));

    ESP_LOGI(TAG, "Probing ES8311...");
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus, ES8311_ADDR, 100));

    ESP_LOGI(TAG, "Probing ES7210...");
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus, ES7210_ADDR, 100));

    i2c_scan_bus(i2c_bus);

    ESP_LOGI(TAG, "Initializing ES8311 codec (DAC)...");
    ESP_ERROR_CHECK(es8311_init_codec());

    ESP_LOGI(TAG, "Initializing ES7210 codec (ADC)...");
    ESP_ERROR_CHECK(es7210_init_adc());


    // Detect whether ES7210 is in TDM mode (SDP_INTERFACE2 bit 1)
    {
        uint8_t sdp12 = 0;
        es7210_read_reg(ES7210_SDP_INTERFACE2_REG12, &sdp12);
        s_es7210_tdm = (sdp12 & 0x02) ? true : false;
    }

    // Initialize LED ring (WS281x) if available
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_LED_COUNT,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags = { .with_dma = 0 },
    };
    if (LED_STRIP_GPIO_PIN >= 0 && LED_STRIP_LED_COUNT > 0) {
        if (led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip) == ESP_OK) {
            ESP_LOGI(TAG, "LED ring initialized on GPIO %d (%d LEDs)", LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT);
            led_strip_clear(s_led_strip);
            led_strip_refresh(s_led_strip);
        } else {
            ESP_LOGW(TAG, "Failed to init LED ring (GPIO %d)", LED_STRIP_GPIO_PIN);
            s_led_strip = NULL;
        }
    }

    // Add TCA9555 port expander device (for keys)
    i2c_device_config_t tca_cfg = {
        .device_address = 0x20,
        .scl_speed_hz = ES_I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(i2c_bus, &tca_cfg, &tca9555_dev) == ESP_OK) {
        // configure both P0 and P1 pins as inputs (regs 0x06 & 0x07)
        uint8_t cfg0[2] = { 0x06, 0xFF };
        uint8_t cfg1[2] = { 0x07, 0xFF };
        i2c_master_transmit(tca9555_dev, cfg0, sizeof(cfg0), 100);
        i2c_master_transmit(tca9555_dev, cfg1, sizeof(cfg1), 100);
        ESP_LOGI(TAG, "TCA9555: configured P0=0xFF P1=0xFF");

        // Read back port input registers and a short register dump to verify device state
        uint8_t read_p0 = tca9555_read_port0();
        uint8_t read_p1 = tca9555_read_port1();
        ESP_LOGI(TAG, "tca9555_readback: P0=0x%02X P1=0x%02X", read_p0, read_p1);

        // Try to read registers 0x00..0x07 directly for diagnosis
        {
            uint8_t regs[8];
            for (uint8_t r = 0; r < 8; ++r) {
                uint8_t reg = r;
                regs[r] = 0xEE;
                if (i2c_master_transmit(tca9555_dev, &reg, 1, 100) == ESP_OK) {
                    if (i2c_master_receive(tca9555_dev, &regs[r], 1, 100) != ESP_OK) {
                        regs[r] = 0xEF;
                    }
                }
            }
            ESP_LOGI(TAG, "tca9555_init_read 0x00..0x07: %02X %02X %02X %02X %02X %02X %02X %02X",
                     regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
        }
    } else {
        ESP_LOGW(TAG, "Failed to add TCA9555 device");
    }


    //
    // --- I2S CHANNEL CREATION ---
    //
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;

    ESP_LOGI(TAG, "Creating I2S channels...");
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));


    //
    // --- I2S RX CONFIG (ES7210 -> ESP32, 32‑bit stereo) ---
    //
    i2s_std_config_t rx_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_LRCLK_PIN,
            .din  = I2S_DIN_PIN,      // ES7210 SDOUT
            .dout = I2S_GPIO_UNUSED,
        },
    };

    ESP_LOGI(TAG, "Initializing I2S RX (ES7210)...");
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &rx_cfg));


    //
    // --- I2S TX CONFIG (ESP32 -> ES8311, 16‑bit mono) ---
    //
    i2s_std_config_t tx_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_LRCLK_PIN,
            .dout = I2S_DOUT_PIN,     // ES8311 DSDIN
            .din  = I2S_GPIO_UNUSED,
        },
    };

    ESP_LOGI(TAG, "Initializing I2S TX (ES8311)...");
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_cfg));

    /* Reconfigure clocks now that channels exist */
    {
        i2s_std_clk_config_t clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        };
        i2s_channel_reconfig_std_clock(rx_handle, &clk_cfg);
        i2s_channel_reconfig_std_clock(tx_handle, &clk_cfg);
    }


    //
    // --- ENABLE CHANNELS ---
    //
    ESP_LOGI(TAG, "Enabling I2S RX...");
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    ESP_LOGI(TAG, "Enabling I2S TX...");
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    
    uint8_t testbuf[64];
    size_t br = 0;
    (void)i2s_channel_read(rx_handle, testbuf, sizeof(testbuf), &br, 100);


    //
    // --- STORE & RETURN HANDLES ---
    //
    i2s_tx_handle = tx_handle;
    i2s_rx_handle = rx_handle;

    *tx_handle_out = tx_handle;
    *rx_handle_out = rx_handle;

    ESP_LOGI(TAG, "Audio driver initialized successfully");
    return ESP_OK;
}

// --------------------------
// Playback helper
// --------------------------
void audio_play_pcm(int16_t *samples, size_t sample_count)
{
    if (!i2s_tx_handle) {
        ESP_LOGE(TAG, "I2S TX not initialized");
        return;
    }

    size_t written = 0;
    esp_err_t err = i2s_channel_write(
        i2s_tx_handle,
        samples,
        sample_count * sizeof(int16_t),
        &written,
        portMAX_DELAY
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(err));
    }
}
