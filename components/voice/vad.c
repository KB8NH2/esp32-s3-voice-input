#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "vad.h"

// ---------- Tunables ----------

// 16 kHz @ 10 ms frames → 160 samples
#define VAD_FRAME_SAMPLES        160

// Energy is average of squares (RMS^2). Adjust as needed.
#define VAD_ENERGY_THRESHOLD     3000000ULL

// Minimum speech duration before firing callback (in samples).
#define VAD_MIN_SPEECH_SAMPLES   12000

// Maximum speech duration before forcing a segment end (in samples).
#define VAD_MAX_SPEECH_SAMPLES   24000

// Number of consecutive non-speech frames required to decide speech has ended.
#define VAD_HANGOVER_FRAMES      80

// Size of internal buffer for accumulating a speech segment.
#define VAD_BUFFER_MAX_SAMPLES   24000

static const char *TAG = "vad";
static vad_callback_t s_vad_cb = NULL;
static int s_in_speech = 0;
static size_t s_accum_samples = 0;
static int16_t s_buffer[VAD_BUFFER_MAX_SAMPLES];
static int s_non_speech_frames = 0;

static int frame_is_speech(const int16_t *frame, size_t n)
{
    uint64_t energy = 0;
    for (size_t i = 0; i < n; i++) {
        int32_t s = frame[i];
        energy += (uint64_t)(s * s);
    }
    energy /= n;
    return (energy > VAD_ENERGY_THRESHOLD);
}

static void vad_reset_state(void)
{
    s_in_speech = 0;
    s_accum_samples = 0;
    s_non_speech_frames = 0;
}

void vad_set_callback(vad_callback_t cb)
{
    s_vad_cb = cb;
}

void vad_reset(void)
{
    vad_reset_state();
}

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

            if (speech) {
                s_non_speech_frames = 0;
            } else {
                s_non_speech_frames++;
            }

            if (s_non_speech_frames >= VAD_HANGOVER_FRAMES &&
                s_accum_samples >= VAD_MIN_SPEECH_SAMPLES) {
                ESP_LOGI(TAG, "END speech (hangover) accum=%u", (unsigned)s_accum_samples);
                if (s_vad_cb) s_vad_cb(s_buffer, s_accum_samples);
                vad_reset_state();
                continue;
            }

            if (s_accum_samples >= VAD_MAX_SPEECH_SAMPLES) {
                ESP_LOGI(TAG, "END speech (max) accum=%u", (unsigned)s_accum_samples);
                if (s_vad_cb) s_vad_cb(s_buffer, s_accum_samples);
                vad_reset_state();
                continue;
            }
        }
    }
}
