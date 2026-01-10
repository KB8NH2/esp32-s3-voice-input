#pragma once
#include <stdint.h>
#include <stddef.h>

#include "driver/i2s_std.h"

extern i2s_chan_handle_t i2s_tx_handle;
extern i2s_chan_handle_t i2s_rx_handle;

esp_err_t audio_driver_init(i2s_chan_handle_t *tx_handle_out,
                            i2s_chan_handle_t *rx_handle_out);

void audio_play_pcm(int16_t *samples, size_t sample_count);

// Key handling
bool key3_pressed(void);
bool debounce_key3(void);

// LED ring control (WS281x)
void led_ring_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_ring_clear(void);

