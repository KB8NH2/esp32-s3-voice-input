#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "vad.h"

// ─────────────────────────────────────────────────────────────
// Tunable parameters
// ─────────────────────────────────────────────────────────────

// 16 kHz @ 10 ms frames → 160 samples
#define VAD_FRAME_SAMPLES        160

// Energy threshold (mean square) tuned to your real data
#define VAD_ENERGY_THRESHOLD     5000000ULL   // 5 million

// Require 5 consecutive frames above threshold to enter speech
#define VAD_ENTER_FRAMES         5

// Hangover frames before ending speech
#define VAD_HANGOVER_FRAMES      40

// Minimum speech duration before firing callback (in samples)
#define VAD_MIN_SPEECH_SAMPLES   12000

// Maximum speech duration before forcing a segment end (in samples)
#define VAD_MAX_SPEECH_SAMPLES   48000

// Speech buffer size
#define VAD_BUFFER_MAX_SAMPLES   48000

// Pre‑roll: 200 ms → 3200 samples
#define VAD_PREROLL_SAMPLES      3200

static const char *TAG = "vad";
static vad_callback_t s_vad_cb = NULL;
static int s_in_speech = 0;
static size_t s_accum_samples = 0;
static int16_t s_buffer[VAD_BUFFER_MAX_SAMPLES];
static int s_non_speech_frames = 0;
static int s_enter_counter = 0;

static int16_t  s_preroll[VAD_PREROLL_SAMPLES];
static size_t   s_preroll_idx = 0;

static int frame_is_speech(const int16_t *frame, size_t n)
{
    uint64_t energy = 0;
    for (size_t i = 0; i < n; i++) {
        int32_t s = frame[i];
        energy += (uint64_t)(s * s);
    }
    energy /= n;
    ESP_LOGV(TAG, "VAD frame energy=%llu", energy);
    return (energy > VAD_ENERGY_THRESHOLD);
}

void vad_set_callback(vad_callback_t cb)
{
    s_vad_cb = cb;
}

// ─────────────────────────────────────────────────────────────
// Helper: write a frame into the circular pre‑roll buffer
// ─────────────────────────────────────────────────────────────
static inline void preroll_write(const int16_t *frame)
{
    memcpy(&s_preroll[s_preroll_idx], frame,
           VAD_FRAME_SAMPLES * sizeof(int16_t));

    s_preroll_idx += VAD_FRAME_SAMPLES;
    if (s_preroll_idx >= VAD_PREROLL_SAMPLES)
        s_preroll_idx = 0;
}


// ─────────────────────────────────────────────────────────────
// Helper: copy pre‑roll into the speech buffer
// ─────────────────────────────────────────────────────────────
static inline void preroll_copy_into_speech(void)
{
    size_t start = s_preroll_idx;  // oldest audio
    size_t first = VAD_PREROLL_SAMPLES - start;

    if (first > VAD_PREROLL_SAMPLES)
        first = VAD_PREROLL_SAMPLES;

    // Copy from start → end
    memcpy(s_buffer, &s_preroll[start], first * sizeof(int16_t));

    // Wrap‑around copy if needed
    if (first < VAD_PREROLL_SAMPLES) {
        memcpy(&s_buffer[first], s_preroll,
               (VAD_PREROLL_SAMPLES - first) * sizeof(int16_t));
    }

    s_accum_samples = VAD_PREROLL_SAMPLES;
}


// ─────────────────────────────────────────────────────────────
// Reset state
// ─────────────────────────────────────────────────────────────
static void vad_reset_state(void)
{
    s_in_speech = 0;
    s_enter_counter = 0;
    s_non_speech_frames = 0;
    s_accum_samples = 0;
}


// ─────────────────────────────────────────────────────────────
// Main VAD processing function
// ─────────────────────────────────────────────────────────────
void vad_process_audio(const int16_t *samples, size_t count)
{
    size_t idx = 0;

    while (idx + VAD_FRAME_SAMPLES <= count) {

        const int16_t *frame = &samples[idx];
        idx += VAD_FRAME_SAMPLES;

        // 1. ALWAYS write frame into pre‑roll buffer
        preroll_write(frame);

        // 2. Compute energy
        int speech = frame_is_speech(frame, VAD_FRAME_SAMPLES);

        // 3. ENTER‑SPEECH LOGIC (not yet in speech)
        if (!s_in_speech) {

            if (speech) {
                s_enter_counter++;
                if (s_enter_counter >= VAD_ENTER_FRAMES) {
                    // Officially enter speech
                    s_in_speech = 1;
                    s_non_speech_frames = 0;

                    // Copy pre‑roll into speech buffer
                    preroll_copy_into_speech();

                    ESP_LOGD(TAG, "ENTER speech (with preroll)");
                }
            } else {
                s_enter_counter = 0;
            }

            if (!s_in_speech)
                continue;   // still waiting for speech
        }

        // 4. IN SPEECH: accumulate frame
        size_t to_copy = VAD_FRAME_SAMPLES;
        if (s_accum_samples + to_copy > VAD_BUFFER_MAX_SAMPLES)
            to_copy = VAD_BUFFER_MAX_SAMPLES - s_accum_samples;

        if (to_copy > 0) {
            memcpy(&s_buffer[s_accum_samples], frame,
                   to_copy * sizeof(int16_t));
            s_accum_samples += to_copy;
        }

        // 5. Hangover logic
        if (speech)
            s_non_speech_frames = 0;
        else
            s_non_speech_frames++;

        if (s_non_speech_frames >= VAD_HANGOVER_FRAMES &&
            s_accum_samples >= VAD_MIN_SPEECH_SAMPLES) {

            ESP_LOGD(TAG, "END speech (hangover) accum=%u",
                     (unsigned)s_accum_samples);

            if (s_vad_cb)
                s_vad_cb(s_buffer, s_accum_samples);

            vad_reset_state();
            continue;
        }

        // 6. Max duration cutoff
        if (s_accum_samples >= VAD_MAX_SPEECH_SAMPLES) {

            ESP_LOGD(TAG, "END speech (max) accum=%u",
                     (unsigned)s_accum_samples);

            if (s_vad_cb)
                s_vad_cb(s_buffer, s_accum_samples);

            vad_reset_state();
            continue;
        }
    }
}