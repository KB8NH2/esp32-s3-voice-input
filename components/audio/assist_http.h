#pragma once
#include <stdint.h>
#include <stddef.h>

void assist_http_init(void);

// PCM: 16 kHz, mono, 16-bit, little-endian
void assist_http_send_utterance(const int16_t *samples, size_t sample_count);

typedef struct {
    char recognized[256];
    char response[256];
} assist_result_t;

typedef void (*assist_result_cb_t)(const char *recognized, const char *response);
void assist_http_set_result_callback(assist_result_cb_t cb);
