// vad.c - simple energy-based VAD with hangover
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "vad.h"

// ---------- Tunables ----------

// 16 kHz @ 10 ms frames → 160 samples
#define VAD_FRAME_SAMPLES        160

// Energy is average of squares (RMS^2).
// For your audio, noise/music ~1e5–2e5, speech ~4e5–1e6+.
// Start at 300k and adjust if needed.
#define VAD_ENERGY_THRESHOLD     3000000ULL

// Minimum speech duration before firing callback (in samples).
// 12000 samples @ 16 kHz ≈ 300 ms.
#define VAD_MIN_SPEECH_SAMPLES   12000

// Maximum speech duration before forcing a segment end (in samples).
// 24000 samples @ 16 kHz ≈ 1.5 s.
#define VAD_MAX_SPEECH_SAMPLES   24000

// Number of consecutive non-speech frames required to decide speech has ended.
// 80 frames @ 10 ms ≈ 800 ms hangover.
#define VAD_HANGOVER_FRAMES      80

// Size of internal buffer for accumulating a speech segment.
// Must be >= VAD_MAX_SPEECH_SAMPLES.
#define VAD_BUFFER_MAX_SAMPLES   24000

// ---------- Types & State ----------

static const char *TAG = "vad";

// Callback type is assumed from your existing code:
// typedef void (*vad_callback_t)(const int16_t *samples, size_t count);
static vad_callback_t s_vad_cb = NULL;

// Are we currently inside a speech segment?
static int s_in_speech = 0;

// Number of samples accumulated for the current speech segment.
static size_t s_accum_samples = 0;

// Accumulation buffer.
static int16_t s_buffer[VAD_BUFFER_MAX_SAMPLES];

// Count of consecutive frames classified as non-speech while in speech mode.
static int s_non_speech_frames = 0;

// ---------- Internal helpers ----------

static int frame_is_speech(const int16_t *frame, size_t n)
{
    uint64_t energy = 0;

    for (size_t i = 0; i < n; i++) {
        int32_t s = frame[i];
        energy += (uint64_t)(s * s);
    }

    energy /= n;

    int speech = (energy > VAD_ENERGY_THRESHOLD);
/*
    float rms = sqrtf((float)energy);
    ESP_LOGI("vad", "energy=%llu, rms=%.1f thresh=%llu speech=%d",
             (unsigned long long)energy,
             rms,
             (unsigned long long)VAD_ENERGY_THRESHOLD,
             speech);
*/    
    return speech;
}

static void vad_reset_state(void)
{
    s_in_speech = 0;
    s_accum_samples = 0;
    s_non_speech_frames = 0;
}

// ---------- Public API ----------

void vad_set_callback(vad_callback_t cb)
{
    s_vad_cb = cb;
}

void vad_reset(void)
{
    vad_reset_state();
}

// Process a block of 16-bit PCM samples (mono, 16 kHz).
// "samples" may contain multiple frames; we stride in VAD_FRAME_SAMPLES.
void vad_process_audio(const int16_t *samples, size_t count)
{
    size_t idx = 0;

    while (idx + VAD_FRAME_SAMPLES <= count) {
        const int16_t *frame = &samples[idx];
        idx += VAD_FRAME_SAMPLES;

        int speech = frame_is_speech(frame, VAD_FRAME_SAMPLES);

        if (!s_in_speech && speech) {
            s_in_speech = 1;
            s_accum_samples = 0;
            s_non_speech_frames = 0;
            ESP_LOGI(TAG, "ENTER speech");
        }

        if (s_in_speech) {
            size_t to_copy = VAD_FRAME_SAMPLES;
            if (s_accum_samples + to_copy > VAD_BUFFER_MAX_SAMPLES) {
                to_copy = VAD_BUFFER_MAX_SAMPLES - s_accum_samples;
            }

            if (to_copy > 0) {
                memcpy(&s_buffer[s_accum_samples], frame, to_copy * sizeof(int16_t));
                s_accum_samples += to_copy;
            }

            // ⭐ THIS WAS MISSING ⭐
            if (speech) {
                s_non_speech_frames = 0;
            } else {
                s_non_speech_frames++;
            }
/*
            ESP_LOGI("vad", "frame: speech=%d ns_frames=%d accum=%u",
                     speech, s_non_speech_frames, (unsigned)s_accum_samples);
*/
            if (s_non_speech_frames >= VAD_HANGOVER_FRAMES &&
                s_accum_samples >= VAD_MIN_SPEECH_SAMPLES) {
                ESP_LOGI(TAG, "END speech (hangover) accum=%u",
                         (unsigned)s_accum_samples);
                if (s_vad_cb) s_vad_cb(s_buffer, s_accum_samples);
                vad_reset_state();
                continue;
            }

            if (s_accum_samples >= VAD_MAX_SPEECH_SAMPLES) {
                ESP_LOGI("vad", "END speech (max) accum=%u",
                         (unsigned)s_accum_samples);
                if (s_vad_cb) s_vad_cb(s_buffer, s_accum_samples);
                vad_reset_state();
                continue;
            }
        }
    }
}