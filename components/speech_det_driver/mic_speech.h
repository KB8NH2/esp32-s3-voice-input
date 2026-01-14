/* mic_speech.h - microphone capture API for speech detection component */
#ifndef MIC_SPEECH_H
#define MIC_SPEECH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void Speech_Init(void);
void mic_start_capture(void);
void mic_stop_capture(void);
int16_t *mic_take_captured_buffer(size_t *samples);
size_t mic_copy_recent_samples(int16_t *dst, size_t max_samples);

#ifdef __cplusplus
}
#endif

#endif // MIC_SPEECH_H
/* mic_speech.h - microphone capture API for speech detection component */
#ifndef MIC_SPEECH_H
#define MIC_SPEECH_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void Speech_Init(void);
void mic_start_capture(void);
void mic_stop_capture(void);
int16_t *mic_take_captured_buffer(size_t *samples);
size_t mic_copy_recent_samples(int16_t *dst, size_t max_samples);

#ifdef __cplusplus
}
#endif

#endif // MIC_SPEECH_H
// Microphone capture API for speech detection component
#pragma once

// Microphone capture API for speech detection component
#pragma once

#include <stdint.h>
#include <stddef.h>

// Initialize microphone and related resources. Safe to call once at startup.
void Speech_Init(void);

// Start and stop capturing audio into the internal capture buffer.
void mic_start_capture(void);
void mic_stop_capture(void);

// Take ownership of the captured PCM buffer. Returns a heap pointer
// that the caller must free. On return, *samples will be set to the
// number of 16-bit PCM samples in the buffer. Returns NULL if no
// captured buffer is available.
int16_t *mic_take_captured_buffer(size_t *samples);

// Lower-level APIs used by VAD/tasks
size_t mic_copy_recent_samples(int16_t *dst, size_t max_samples);
#include <stdint.h>
