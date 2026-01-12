#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mic_speech.h"
#include "vad.h"
#include "audio_driver.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <math.h>
#include "esp_heap_caps.h"

static const char *TAG = "mic_speech";

static void mic_task(void *arg);

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

// Capture (push-to-talk)
static bool s_capture_enabled = false;
static int16_t *s_capture_buf = NULL;
static size_t s_capture_samples = 0;
static size_t s_capture_capacity = 0;
static SemaphoreHandle_t s_capture_mutex = NULL;
static const size_t CAPTURE_INITIAL_SAMPLES = 16000; // ~1s @ 16k
static int64_t s_capture_start_us = 0;

void Speech_Init(void)
{
    ESP_LOGI(TAG, "Initializing microphone subsystem");

    // Initialize audio hardware (ES8311 + I2S RX/TX)
    ESP_ERROR_CHECK(audio_driver_init(&tx_handle, &rx_handle));

    // Initialize VAD (callback set later in main.c)
    // vad_init(NULL);

    // Create capture mutex BEFORE starting task to avoid races
    s_capture_mutex = xSemaphoreCreateMutex();

    // Start microphone capture task
    xTaskCreatePinnedToCore(
        mic_task,
        "mic_task",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Microphone subsystem initialized");
}

void mic_start_capture(void)
{
    if (!s_capture_mutex) {
        ESP_LOGW(TAG, "mic_start_capture called but mutex not initialized");
        return;
    }
    if (xSemaphoreTake(s_capture_mutex, portMAX_DELAY) == pdTRUE) {
        s_capture_samples = 0;
        s_capture_enabled = true;
        s_capture_start_us = esp_timer_get_time();
        ESP_LOGI(TAG, "mic_start_capture: enabled, samples reset");
        xSemaphoreGive(s_capture_mutex);
    }
}

void mic_stop_capture(void)
{
    if (!s_capture_mutex) {
        ESP_LOGW(TAG, "mic_stop_capture called but mutex not initialized");
        return;
    }
    if (xSemaphoreTake(s_capture_mutex, portMAX_DELAY) == pdTRUE) {
        s_capture_enabled = false;
        int64_t now = esp_timer_get_time();
        if (s_capture_start_us > 0 && now > s_capture_start_us) {
            double secs = (now - s_capture_start_us) / 1000000.0;
            double rate = secs > 0 ? (double)s_capture_samples / secs : 0.0;
            ESP_LOGI(TAG, "mic_stop_capture: samples=%u duration=%.3fs approx_rate=%.1f Hz",
                     (unsigned)s_capture_samples, secs, rate);
        }
        s_capture_start_us = 0;
        ESP_LOGI(TAG, "mic_stop_capture: disabled");
        xSemaphoreGive(s_capture_mutex);
    }
}

int16_t *mic_take_captured_buffer(size_t *out_samples)
{
    if (!s_capture_mutex) {
        ESP_LOGW(TAG, "mic_take_captured_buffer called but mutex not initialized");
        return NULL;
    }
    int16_t *out = NULL;
    if (xSemaphoreTake(s_capture_mutex, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "mic_take_captured_buffer: internal samples=%u capacity=%u", (unsigned)s_capture_samples, (unsigned)s_capture_capacity);
        if (s_capture_samples > 0) {
            // Transfer ownership of internal buffer to caller to avoid extra allocation/copy.
            out = s_capture_buf;
            if (out_samples) *out_samples = s_capture_samples;
            ESP_LOGI(TAG, "mic_take_captured_buffer: returning ownership of %u samples (%u bytes)", (unsigned)s_capture_samples, (unsigned)(s_capture_samples * sizeof(int16_t)));
            // Clear internal state but do NOT free out (caller owns it)
            s_capture_buf = NULL;
            s_capture_capacity = 0;
            s_capture_samples = 0;
        } else {
            if (out_samples) *out_samples = 0;
            // No samples: free any allocated internal buffer
            free(s_capture_buf);
            s_capture_buf = NULL;
            s_capture_capacity = 0;
            s_capture_samples = 0;
        }
        xSemaphoreGive(s_capture_mutex);
    }
    return out;
}

static inline int16_t es7210_downmix_mono(const int16_t *raw)
{
    // raw[0] = MIC3 (AEC ref)
    // raw[1] = MIC1
    // raw[2] = MIC4 (ground)
    // raw[3] = MIC2

    int32_t mic1 = raw[1];
    int32_t mic2 = raw[3];

    return (int16_t)((mic1 + mic2) / 2);
}
static void mic_task(void *arg)
{
    static int16_t vad_buf[160];
    static size_t vad_index = 0;

    const size_t frame_bytes = 512;   // 512 bytes = 128 samples @ 32‑bit
    uint8_t *buf = malloc(frame_bytes);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate mic buffer");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Mic task started");

    for (;;) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(
            rx_handle,
            buf,
            frame_bytes,
            &bytes_read,
            portMAX_DELAY
        );

        if (ret != ESP_OK || bytes_read != frame_bytes) {
            ESP_LOGE(TAG, "I2S read error: %s, bytes=%u",
                     esp_err_to_name(ret), (unsigned)bytes_read);
            continue;
        }

        // ----------------------------------------------------
        // ES7210: 4-channel TDM packed into 32-bit I2S words
        //
        // samples32[i] contains ONE 32-bit slot:
        //   bits 31..16 = PCM sample
        //   bits 15..0  = unused
        //
        // The ES7210 outputs channels in this order:
        //   slot 0 → MIC3 (AEC ref)
        //   slot 1 → MIC1
        //   slot 2 → MIC4 (ground)
        //   slot 3 → MIC2
        //
        // We downmix MIC1 + MIC2 → mono
        // ----------------------------------------------------

        int32_t *samples32 = (int32_t *)buf;
        int total_slots = bytes_read / sizeof(int32_t);   // should be 128
        bool tdm = audio_driver_es7210_is_tdm();
        int channels_per_frame = tdm ? 4 : 2;
        int frames = total_slots / channels_per_frame;    // frames per read

        (void)total_slots; (void)frames; (void)samples32;

        int16_t pcm16[64];  // support up to 64 mono samples per 512-byte read

        int16_t *pcm_ptr = pcm16;

        for (int f = 0; f < frames; f++) {
            int32_t s_mic1, s_mic2;
            if (tdm) {
                s_mic1 = samples32[f * 4 + 1] >> 16;   // MIC1
                s_mic2 = samples32[f * 4 + 3] >> 16;   // MIC2
            } else {
                // Standard I2S stereo: assume left=MIC1, right=MIC2
                s_mic1 = samples32[f * 2 + 0] >> 16;
                s_mic2 = samples32[f * 2 + 1] >> 16;
            }

            // Downmix
            int32_t mono = (s_mic1 + s_mic2) / 2;

            // Apply your 24 dB gain
            mono *= 8;

            // Clip
            if (mono > 32767) mono = 32767;
            if (mono < -32768) mono = -32768;

            pcm_ptr[f] = (int16_t)mono;
        }

        // ----------------------------------------------------
        // Compute RMS + peak (optional)
        // ----------------------------------------------------
        int64_t sum_sq = 0;
        int16_t peak = 0;

        for (int i = 0; i < frames; i++) {
            int16_t s = pcm16[i];
            sum_sq += (int64_t)s * s;
            if (abs(s) > peak) peak = abs(s);
        }
    
        // ----------------------------------------------------
        // Feed mono PCM into VAD and optional capture buffer
        // ----------------------------------------------------
        for (int i = 0; i < frames; i++) {
            int16_t s = pcm16[i];

            // VAD
            vad_buf[vad_index++] = s;
            if (vad_index == 160) {
                vad_process_audio(vad_buf, 160);
                vTaskDelay(1);
                vad_index = 0;
            }

            // Capture (push-to-talk)
            if (s_capture_mutex && xSemaphoreTake(s_capture_mutex, 0) == pdTRUE) {
                if (s_capture_enabled) {
                    if (s_capture_samples + 1 > s_capture_capacity) {
                        size_t new_cap = s_capture_capacity ? s_capture_capacity * 2 : CAPTURE_INITIAL_SAMPLES;
                        int16_t *nb = realloc(s_capture_buf, new_cap * sizeof(int16_t));
                        if (nb) {
                            s_capture_buf = nb;
                            s_capture_capacity = new_cap;
                        }
                    }
                    if (s_capture_samples < s_capture_capacity) {
                        s_capture_buf[s_capture_samples++] = s;
                    }
                }
                xSemaphoreGive(s_capture_mutex);
            }
        }
    }

    // never reached
}