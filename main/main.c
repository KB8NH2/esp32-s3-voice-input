#include "wifi.h"
#include "mic_speech.h"
#include "vad.h"
#include "stt.h"
#include "conversation_api.h"
#include "piper_tts.h"
#include "audio_driver.h"

#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "main";

// PTT event queue
typedef enum { PTT_EVENT_NONE = 0, PTT_EVENT_VAD_END } ptt_event_t;
static QueueHandle_t s_ptt_event_queue = NULL;
static volatile int s_ptt_listening = 0;

// VAD -> PTT notifier
static void vad_ptt_notify_cb(const int16_t *samples, size_t count)
{
    (void)samples; (void)count;
    if (s_ptt_event_queue) {
        ptt_event_t ev = PTT_EVENT_VAD_END;
        xQueueSend(s_ptt_event_queue, &ev, 0);
    }
}

// Forward declarations
static void on_stt_result(const char *text);
static void on_conversation_reply(const char *reply_text);
static void ptt_task(void *arg);

// Argument passed to the STT sender task
struct stt_task_arg {
    int16_t *buf;
    size_t samples;
};

static void stt_sender_task(void *v)
{
    struct stt_task_arg *a = (struct stt_task_arg *)v;
    ESP_LOGI(TAG, "stt_task: calling stt_send_wav_multipart samples=%u", (unsigned)a->samples);
    char *resp = stt_send_wav_multipart(a->buf, a->samples);
    free(a->buf);
    if (resp) {
        on_stt_result(resp);
        free(resp);
    } else {
        ESP_LOGW(TAG, "stt_send_wav_multipart returned NULL");
    }
    free(a);
    vTaskDelete(NULL);
}

static void ptt_stop_and_send(void)
{
    ESP_LOGI(TAG, "PTT: ptt_stop_and_send invoked");
    mic_stop_capture();
    led_ring_clear();
    size_t samples = 0;
    int16_t *buf = mic_take_captured_buffer(&samples);
    ESP_LOGI(TAG, "PTT: captured samples=%u", (unsigned)samples);
    if (buf && samples > 0) {
        // Offload STT upload to its own task to avoid stack exhaustion
        ESP_LOGI(TAG, "PTT: spawning stt sender task for %u samples", (unsigned)samples);
        struct stt_task_arg *arg = malloc(sizeof(*arg));
        if (!arg) {
            ESP_LOGE(TAG, "Failed to allocate stt task arg");
            free(buf);
            return;
        }
        arg->buf = buf;
        arg->samples = samples;
        BaseType_t ok = xTaskCreatePinnedToCore(stt_sender_task, "stt_task", 16384, arg, 5, NULL, 1);
        if (ok != pdTRUE) {
            ESP_LOGE(TAG, "Failed to create stt task");
            free(arg->buf);
            free(arg);
        }
    } else {
        ESP_LOGW(TAG, "No audio captured to send");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Booting voice endpoint firmware");

    // 1. Connect Wi-Fi
    wifi_init();
    wifi_wait_connected();

    // 2. Initialize void tca9555 GPIO expander
    // (done inside audio_driver_init)

    // 3. Initialize audio (mic + speaker)
    Speech_Init();

    // create PTT event queue and register VAD callback to notify PTT task on VAD END
    s_ptt_event_queue = xQueueCreate(4, sizeof(ptt_event_t));
    vad_set_callback(vad_ptt_notify_cb);

    // 4. Start push-to-talk state machine task
    xTaskCreatePinnedToCore(ptt_task, "ptt_task", 4096, NULL, 5, NULL, 1);

    // 5. Initialize Conversation API
    conversation_init("http://192.168.1.154:8000/conversation");

    // 6. Register Conversation API callback
    conversation_set_callback(on_conversation_reply);

    ESP_LOGI(TAG, "System ready");
}

/********************************************************************
 *  CALLBACK CHAIN
 ********************************************************************/

// Push-to-talk task: polls key3 and captures audio while pressed
static void ptt_task(void *arg)
{
    enum { STATE_IDLE = 0, STATE_LISTENING, STATE_PROCESSING, STATE_PLAYING_TTS } state = STATE_IDLE;

    bool last_pressed = false;

    for (;;) {
        // check for VAD notifications
        if (s_ptt_event_queue) {
            ptt_event_t ev;
            if (xQueueReceive(s_ptt_event_queue, &ev, 0) == pdTRUE) {
                if (ev == PTT_EVENT_VAD_END && state == STATE_LISTENING) {
                    ESP_LOGI(TAG, "PTT: VAD END detected, stopping and sending");
                    s_ptt_listening = 0;
                    ptt_stop_and_send();
                    state = STATE_PROCESSING;
                }
            }
        }
        bool pressed = debounce_key3();
        bool pressed_edge = pressed && !last_pressed; // detect press event
        last_pressed = pressed;

        switch (state) {
        case STATE_IDLE:
            if (pressed_edge) {
                ESP_LOGI(TAG, "PTT: start listening (toggle)");
                mic_start_capture();
                led_ring_set_color(0, 0, 64); // dim blue
                s_ptt_listening = 1;
                state = STATE_LISTENING;
            }
            break;

        case STATE_LISTENING:
            if (pressed_edge) {
                ESP_LOGI(TAG, "PTT: stop listening (toggle), sending audio");
                s_ptt_listening = 0;
                ptt_stop_and_send();
                state = STATE_PROCESSING;
            }
            break;

        case STATE_PROCESSING:
            // For now, transition back to idle. Conversation / TTS callbacks
            // could drive further states.
            state = STATE_IDLE;
            break;

        case STATE_PLAYING_TTS:
            state = STATE_IDLE;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Whisper STT → HA Conversation
static void on_stt_result(const char *text)
{
    if (strlen(text) == 0) {
        ESP_LOGI(TAG, "STT: empty result");
        return;
    }
    ESP_LOGI(TAG, "STT: \"%s\"", text);
    conversation_send(text);
}

// Conversation → HA TTS
static void on_conversation_reply(const char *reply_text)
{
    ESP_LOGI(TAG, "Conversation Server Reply: \"%s\"", reply_text);
    //piper_tts_synthesize(reply_text);
}
