#include "audio_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"

// --------------------------
// I2C config
// --------------------------
#define ES_I2C_PORT        0
#include "led_strip.h"
#include "../hardeware_driver/bsp_board.h"
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

i2s_chan_handle_t i2s_tx_handle = NULL;   // speaker
i2s_chan_handle_t i2s_rx_handle = NULL;   // mic

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
    ESP_LOGD(TAG, "Starting I2C scan on this bus...");
    for (int addr = 0x03; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            ESP_LOGD(TAG, "I2C device found at 0x%02X", addr);
        }
    }
    ESP_LOGD(TAG, "I2C scan complete");
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
    ESP_LOGD(TAG, "Initializing ES8311 DAC...");

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
    ESP_LOGD(TAG, "ES8311 REG02 (ADC power) = 0x%02X", reg);
    es8311_read_reg(0x1A, &reg);
    ESP_LOGD(TAG, "ES8311 REG1A (MICBIAS) = 0x%02X", reg);
    es8311_read_reg(0x23, &reg);
    ESP_LOGD(TAG, "ES8311 REG23 (PGA gain) = 0x%02X", reg);

    ESP_LOGD(TAG, "ES8311 initialized");
    return ESP_OK;
}

// --------------------------
// ES7210 ADC init (mic)
// Mirrors the key behavior of:
//   - es7210_set_bits (bits per sample)
//   - es7210_config_fmt (I2S normal)
//   - mic select + gain for MIC1/MIC2
//   - clock off / power / bias setup
// --------------------------
static esp_err_t es7210_init_adc(void)
{
    ESP_LOGD(TAG, "Initializing ES7210 ADC...");

    // Soft reset sequence (similar to start/open behavior)
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_RESET_REG00, 0x71));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_RESET_REG00, 0x41));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Slave mode: clear master bit in MODE_CONFIG_REG08 (mask 0x01 -> 0x00)
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MODE_CONFIG_REG08, 0x01, 0x00));

    // MCLK from PAD: clear bit7 in MASTER_CLK_REG03
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MASTER_CLK_REG03, 0x80, 0x00));

    // Analog power + MICBIAS + OSR + main clock as seen in es7210_open()
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_ANALOG_REG40, 0x43));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC12_BIAS_REG41, 0x70));  // 2.87V
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC34_BIAS_REG42, 0x70));  // 2.87V
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_OSR_REG07, 0x20));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MAINCLK_REG02, 0xC1));     // clear state, coeff preset

    // I2S interface: bits per sample (32) via es7210_set_bits logic
    uint8_t iface = 0;
    ESP_ERROR_CHECK(es7210_read_reg(ES7210_SDP_INTERFACE1_REG11, &iface));
    iface &= 0x1F;          // keep low 5 bits
    iface |= 0x80;          // bits = 32 -> add 0x80
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_SDP_INTERFACE1_REG11, iface));
    ESP_LOGD(TAG, "ES7210: set bits per sample = 32");

    // Format: I2S normal (ES_I2S_NORMAL) via es7210_config_fmt logic
    ESP_ERROR_CHECK(es7210_read_reg(ES7210_SDP_INTERFACE1_REG11, &iface));
    iface &= 0xFC;          // clear format bits
    iface |= 0x00;          // I2S normal
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_SDP_INTERFACE1_REG11, iface));

    // Disable TDM mode: SDP_INTERFACE2_REG12 = 0x00
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_SDP_INTERFACE2_REG12, 0x00));

    // Power-up path similar to es7210_start():
    // - CLOCK_OFF_REG01: use initial value as "off_reg", then clear the clock-off bits we care about
    uint8_t clock_off = 0;
    ESP_ERROR_CHECK(es7210_read_reg(ES7210_CLOCK_OFF_REG01, &clock_off));
    // Clear bits that gate MIC1/2 clocks (0x0B mask used in HAL)
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_CLOCK_OFF_REG01, 0x0B, 0x00));

    // POWER_DOWN_REG06 = 0 -> power up
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_POWER_DOWN_REG06, 0x00));

    // Analog power again as in start()
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_ANALOG_REG40, 0x43));

    // Individual MIC power regs: 0x08 as seen in es7210_start()
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC1_POWER_REG47, 0x08));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC2_POWER_REG48, 0x08));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC3_POWER_REG49, 0x08));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC4_POWER_REG4A, 0x08));

    // Global MIC12/MIC34 power
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC12_POWER_REG4B, 0xFF));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC34_POWER_REG4C, 0xFF));

    // Enable MIC1 & MIC2 with gain, mirroring es7210_mic_select/_set_channel_gain behavior
    // Enable MIC1
    ESP_LOGD(TAG, "Enable ES7210_INPUT_MIC1");
    // Clear clock-off bits for MIC1/2 (0x0B mask)
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_CLOCK_OFF_REG01, 0x0B, 0x00));
    ESP_ERROR_CHECK(es7210_write_reg(ES7210_MIC12_POWER_REG4B, 0x00));   // power MIC1/2 path
    // MIC1 gain enable bit (0x10) and gain value (0x0F mask)
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MIC1_GAIN_REG43, 0x10, 0x10)); // enable gain
    ESP_ERROR_CHECK(es7210_update_reg_bit(ES7210_MIC1_GAIN_REG43, 0x0F, 0x0C)); // ~12 dB

    // Enable MIC2
    ESP_LOGD(TAG, "Enable ES7210_INPUT_MIC2");
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
    ESP_LOGD(TAG, "ES7210 CLOCK_OFF=0x%02X PWDN=0x%02X ANALOG40=0x%02X", r1, r2, r40);

    uint8_t mic1g, mic2g;
    es7210_read_reg(ES7210_MIC1_GAIN_REG43, &mic1g);
    es7210_read_reg(ES7210_MIC2_GAIN_REG44, &mic2g);
    ESP_LOGD(TAG, "ES7210 MIC1_GAIN=0x%02X MIC2_GAIN=0x%02X", mic1g, mic2g);

    ESP_LOGD(TAG, "ES7210 ADC initialized");
    return ESP_OK;
}

// --------------------------
// TCA9555 GPIO expander init
// --------------------------
#define TCA9555_ADDR  0x20
void tca9555_init(i2c_port_t port) {
    uint8_t config_reg = 0x07;  // Port 1 configuration register
    uint8_t config_val = 0xFF;  // All bits input

    if (!tca9555_dev) {
        i2c_device_config_t cfg = { .device_address = TCA9555_ADDR, .scl_speed_hz = ES_I2C_FREQ_HZ };
        if (i2c_master_bus_add_device(i2c_bus, &cfg, &tca9555_dev) != ESP_OK) return;
    }

    uint8_t data[2] = { config_reg, config_val };
    i2c_master_transmit(tca9555_dev, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

// Read Port 1
uint8_t tca9555_read_port1(i2c_port_t port) {
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

bool key3_pressed(void) {
    uint8_t port1 = tca9555_read_port1(I2C_NUM_0);
    return !(port1 & (1 << 1));  // P1_1
}

bool debounce_key3(void) {
    static uint32_t last_change = 0;
    static bool last_state = false;

    bool current = key3_pressed();
    uint32_t now = xTaskGetTickCount();

    if (current != last_state && (now - last_change) > pdMS_TO_TICKS(100)) {
        last_state = current;
        last_change = now;
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

    ESP_LOGD(TAG, "Creating I2C bus...");
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t es8311_cfg = {
        .device_address = ES8311_ADDR,
        .scl_speed_hz = ES_I2C_FREQ_HZ,
    };
    i2c_device_config_t es7210_cfg = {
        .device_address = ES7210_ADDR,
        .scl_speed_hz = ES_I2C_FREQ_HZ,
    };

    ESP_LOGD(TAG, "Adding ES8311 device...");
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &es8311_cfg, &es8311_dev));

    ESP_LOGD(TAG, "Adding ES7210 device...");
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &es7210_cfg, &es7210_dev));

    ESP_LOGD(TAG, "Probing ES8311...");
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus, ES8311_ADDR, 100));

    ESP_LOGD(TAG, "Probing ES7210...");
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus, ES7210_ADDR, 100));

    i2c_scan_bus(i2c_bus);

    ESP_LOGD(TAG, "Initializing ES8311 codec (DAC)...");
    ESP_ERROR_CHECK(es8311_init_codec());

    ESP_LOGD(TAG, "Initializing ES7210 codec (ADC)...");
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
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags = { .with_dma = 0 },
    };
    if (LED_STRIP_GPIO_PIN >= 0 && LED_STRIP_LED_COUNT > 0) {
        if (led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip) == ESP_OK) {
            ESP_LOGD(TAG, "LED ring initialized on GPIO %d (%d LEDs)", LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT);
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
        // configure all P1 pins as inputs (reg 0x07)
        uint8_t data[2] = { 0x07, 0xFF };
        i2c_master_transmit(tca9555_dev, data, sizeof(data), 100);
    } else {
        ESP_LOGW(TAG, "Failed to add TCA9555 device");
    }


    //
    // --- I2S CHANNEL CREATION ---
    //
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;

    ESP_LOGD(TAG, "Creating I2S channels...");
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

    ESP_LOGD(TAG, "Initializing I2S RX (ES7210)...");
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

    ESP_LOGD(TAG, "Initializing I2S TX (ES8311)...");
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
    ESP_LOGD(TAG, "Enabling I2S RX...");
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    ESP_LOGD(TAG, "Enabling I2S TX...");
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

    ESP_LOGD(TAG, "Audio driver initialized successfully");
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