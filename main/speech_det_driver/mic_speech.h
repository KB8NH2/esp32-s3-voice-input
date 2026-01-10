#pragma once

#include <stdint.h>
#include <stddef.h>

void Speech_Init(void);

// Push-to-talk capture APIs
void mic_start_capture(void);
void mic_stop_capture(void);
// Caller takes ownership of returned buffer and must free(); returns NULL if no samples.
int16_t *mic_take_captured_buffer(size_t *out_samples);