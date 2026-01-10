#pragma once
#include <stdint.h>
#include <stddef.h>

typedef void (*tts_pcm_callback_t)(const int16_t *pcm, size_t samples);

void piper_tts_init(const char *url);
void piper_tts_set_callback(tts_pcm_callback_t cb);
void piper_tts_synthesize(const char *text);