#pragma once
#include <stdint.h>
#include <stddef.h>

typedef void (*vad_callback_t)(const int16_t *audio, size_t samples);

void vad_init(vad_callback_t cb);
void vad_set_callback(vad_callback_t cb);
void vad_process_audio(const int16_t *samples, size_t count);
