/* audio_driver.h - public API for audio playback/capture */
#ifndef AUDIO_DRIVER_H
#define AUDIO_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif



esp_err_t audio_driver_init(i2s_chan_handle_t *tx_handle_out,
                            i2s_chan_handle_t *rx_handle_out);

// Return the I2C master bus handle (for use by other components like tca9555_driver)
i2c_master_bus_handle_t audio_driver_get_i2c_bus(void);

void audio_play_pcm(int16_t *samples, size_t sample_count);

// Return true if ES7210 is configured in TDM mode (4 slots per LRCK)
bool audio_driver_es7210_is_tdm(void);

// Key handling
bool key3_pressed(void);
bool debounce_key3(void);
bool key1_pressed(void);
bool debounce_key1(void);
bool key2_pressed(void);
bool debounce_key2(void);

// LED ring control (WS281x)
void led_ring_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_ring_clear(void);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_DRIVER_H
