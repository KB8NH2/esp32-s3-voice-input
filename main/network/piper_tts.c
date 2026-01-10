#include "piper_tts.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "piper_tts";

static tts_pcm_callback_t s_tts_cb = NULL;
static char s_tts_url[128];

static int16_t *s_pcm_buf = NULL;
static size_t s_pcm_samples = 0;
static size_t s_pcm_capacity = 0;

void piper_tts_init(const char *url)
{
    strncpy(s_tts_url, url, sizeof(s_tts_url));
    s_tts_url[sizeof(s_tts_url) - 1] = '\0';
    ESP_LOGI(TAG, "Piper TTS URL: %s", s_tts_url);
}

void piper_tts_set_callback(tts_pcm_callback_t cb)
{
    s_tts_cb = cb;
}

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (evt->data_len > 0) {
            size_t new_samples = evt->data_len / sizeof(int16_t);
            size_t needed = s_pcm_samples + new_samples;

            if (needed > s_pcm_capacity) {
                size_t new_cap = (needed + 1023) & ~1023;
                int16_t *new_buf = realloc(s_pcm_buf, new_cap * sizeof(int16_t));
                if (!new_buf) {
                    ESP_LOGE(TAG, "TTS: realloc failed");
                    return ESP_FAIL;
                }
                s_pcm_buf = new_buf;
                s_pcm_capacity = new_cap;
            }

            memcpy(&s_pcm_buf[s_pcm_samples], evt->data, evt->data_len);
            s_pcm_samples += new_samples;
        }
        break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "TTS: received %u samples", (unsigned)s_pcm_samples);
        if (s_tts_cb && s_pcm_samples > 0) {
            s_tts_cb(s_pcm_buf, s_pcm_samples);
        }
        s_pcm_samples = 0;
        break;

    default:
        break;
    }
    return ESP_OK;
}

void piper_tts_synthesize(const char *text)
{
    ESP_LOGI(TAG, "Requesting TTS for: \"%s\"", text);

    esp_http_client_config_t cfg = {
        .url = s_tts_url,
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    char body[512];
    snprintf(body, sizeof(body),
             "{\"text\": \"%s\"}", text);

    esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TTS request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}