#include "assist_http.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "mbedtls/base64.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ASSIST_HTTP";

// TODO: put your HA long-lived token here
static const char *HA_TOKEN = "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJhMGViYTM2YjY4YzQ0NGZiOGM3Zjk2MjEyMWM0YjI2MSIsImlhdCI6MTc2NzU2NDM2MywiZXhwIjoyMDgyOTI0MzYzfQ.sEjAf6TBIPfMDYJ9L7iFuhKulS78ASeR2-U8EY5W13M";

// Hostname you gave:
static const char *HA_HOST = "http://192.168.1.154:8123";

// Simple WAV header builder for 16-bit PCM mono
static void build_wav_header(uint8_t *hdr, uint32_t sample_rate, uint32_t sample_count)
{
    uint32_t byte_rate = sample_rate * 2;
    uint32_t data_size = sample_count * 2;
    uint32_t riff_size = 36 + data_size;

    memcpy(hdr, "RIFF", 4);
    hdr[4] = (uint8_t)(riff_size & 0xFF);
    hdr[5] = (uint8_t)((riff_size >> 8) & 0xFF);
    hdr[6] = (uint8_t)((riff_size >> 16) & 0xFF);
    hdr[7] = (uint8_t)((riff_size >> 24) & 0xFF);
    memcpy(hdr + 8, "WAVEfmt ", 8);

    // fmt chunk
    hdr[16] = 16; hdr[17] = 0; hdr[18] = 0; hdr[19] = 0; // Subchunk1Size = 16
    hdr[20] = 1; hdr[21] = 0; // AudioFormat = PCM
    hdr[22] = 1; hdr[23] = 0; // NumChannels = 1
    hdr[24] = (uint8_t)(sample_rate & 0xFF);
    hdr[25] = (uint8_t)((sample_rate >> 8) & 0xFF);
    hdr[26] = (uint8_t)((sample_rate >> 16) & 0xFF);
    hdr[27] = (uint8_t)((sample_rate >> 24) & 0xFF);
    hdr[28] = (uint8_t)(byte_rate & 0xFF);
    hdr[29] = (uint8_t)((byte_rate >> 8) & 0xFF);
    hdr[30] = (uint8_t)((byte_rate >> 16) & 0xFF);
    hdr[31] = (uint8_t)((byte_rate >> 24) & 0xFF);
    hdr[32] = 2; hdr[33] = 0; // BlockAlign = 2 (16-bit mono)
    hdr[34] = 16; hdr[35] = 0; // BitsPerSample = 16

    memcpy(hdr + 36, "data", 4);
    hdr[40] = (uint8_t)(data_size & 0xFF);
    hdr[41] = (uint8_t)((data_size >> 8) & 0xFF);
    hdr[42] = (uint8_t)((data_size >> 16) & 0xFF);
    hdr[43] = (uint8_t)((data_size >> 24) & 0xFF);
}

void assist_http_init(void)
{
    // Nothing yet; placeholder for any future setup
}

void assist_http_send_utterance(const int16_t *samples, size_t sample_count)
{
    ESP_LOGD(TAG, "Sending utterance: %u samples", (unsigned)sample_count);

    // 1. Base64 encode PCM16
    size_t pcm_bytes = sample_count * sizeof(int16_t);
    size_t b64_len = 0;

    // First call: get required output length
    mbedtls_base64_encode(NULL, 0, &b64_len,
                        (const unsigned char *)samples,
                        pcm_bytes);

    char *b64_audio = malloc(b64_len + 1);
    if (!b64_audio) {
        ESP_LOGE(TAG, "Failed to allocate base64 buffer");
        return;
    }

    // Second call: actually encode
    int ret = mbedtls_base64_encode((unsigned char *)b64_audio, b64_len, &b64_len,
                                    (const unsigned char *)samples,
                                    pcm_bytes);

    if (ret != 0) {
        ESP_LOGE(TAG, "Base64 encode failed: %d", ret);
        free(b64_audio);
        return;
    }

    b64_audio[b64_len] = '\0';

    // 2. Build JSON body
    char *json = malloc(b64_len + 256);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        free(b64_audio);
        return;
    }

    snprintf(json, b64_len + 256,
        "{"
        "\"start_stage\":\"stt\"," 
        "\"end_stage\":\"intent\"," 
        "\"input\":{"
            "\"sample_rate\":16000,"
            "\"channels\":1,"
            "\"audio\":\"%s\""
        "}"
        "}",
        b64_audio
    );

    free(b64_audio);

    // 3. Send HTTP POST
    char url[256];
    snprintf(url, sizeof(url), "%s/api/assist_pipeline", HA_HOST);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        free(json);
        return;
    }

    esp_http_client_set_header(client, "Authorization", HA_TOKEN);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_http_client_set_post_field(client, json, strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGD(TAG, "Assist HTTP status: %d", status);
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(json);
}

// Optional LVGL callback support (stub for now)
typedef void (*assist_result_cb_t)(const char *recognized, const char *response);
static assist_result_cb_t result_callback = NULL;

void assist_http_set_result_callback(assist_result_cb_t cb)
{
    result_callback = cb;
}

// Call this later when we parse HA's response
static void assist_http_invoke_callback(const char *recognized, const char *response)
{
    if (result_callback) {
        result_callback(recognized, response);
    }
}
