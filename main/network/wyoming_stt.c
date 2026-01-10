#include "esp_http_client.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "stt";

// Build a 44-byte WAV header for PCM16 mono 16 kHz
static void build_wav_header(uint8_t *header, uint32_t pcm_data_size) {
    uint32_t file_size = pcm_data_size + 36;

    memcpy(header, "RIFF", 4);
    header[4] = file_size & 0xff;
    header[5] = (file_size >> 8) & 0xff;
    header[6] = (file_size >> 16) & 0xff;
    header[7] = (file_size >> 24) & 0xff;

    memcpy(header + 8, "WAVE", 4);
    memcpy(header + 12, "fmt ", 4);

    header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0; // Subchunk1Size
    header[20] = 1; header[21] = 0;   // PCM
    header[22] = 1; header[23] = 0;   // Mono

    // Sample rate = 16000
    header[24] = 0x80; header[25] = 0x3E; header[26] = 0; header[27] = 0;

    uint32_t byte_rate = 16000 * 2;
    header[28] = byte_rate & 0xff;
    header[29] = (byte_rate >> 8) & 0xff;
    header[30] = (byte_rate >> 16) & 0xff;
    header[31] = (byte_rate >> 24) & 0xff;

    header[32] = 2; header[33] = 0;   // BlockAlign
    header[34] = 16; header[35] = 0;  // BitsPerSample

    memcpy(header + 36, "data", 4);
    header[40] = pcm_data_size & 0xff;
    header[41] = (pcm_data_size >> 8) & 0xff;
    header[42] = (pcm_data_size >> 16) & 0xff;
    header[43] = (pcm_data_size >> 24) & 0xff;
}

char *stt_send_wav_multipart(const int16_t *pcm_data, int pcm_samples) {
    const char *boundary = "----ESP32Boundary";
    const char *url = "http://192.168.1.154:10300/asr";

    int pcm_bytes = pcm_samples * 2;

    // Build WAV header
    uint8_t wav_header[44];
    build_wav_header(wav_header, pcm_bytes);

    // Multipart header
    char form_header[256];
    int header_len = snprintf(form_header, sizeof(form_header),
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"audio_file\"; filename=\"audio.wav\"\r\n"
        "Content-Type: audio/wav\r\n\r\n",
        boundary);

    // Multipart footer
    char form_footer[64];
    int footer_len = snprintf(form_footer, sizeof(form_footer),
        "\r\n--%s--\r\n",
        boundary);

    // Allocate full upload buffer
    int total_len = header_len + 44 + pcm_bytes + footer_len;
    uint8_t *upload_buf = malloc(total_len);
    if (!upload_buf) {
        ESP_LOGE(TAG, "malloc failed");
        return NULL;
    }

    // Build multipart body
    uint8_t *p = upload_buf;
    memcpy(p, form_header, header_len); p += header_len;
    memcpy(p, wav_header, 44); p += 44;
    memcpy(p, pcm_data, pcm_bytes); p += pcm_bytes;
    memcpy(p, form_footer, footer_len);

    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    char content_type[128];
    snprintf(content_type, sizeof(content_type),
             "multipart/form-data; boundary=%s", boundary);

    esp_http_client_set_post_field(client, (const char *)upload_buf, total_len);

    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_header(client, "Accept", "application/json");

    char content_length[32];
    snprintf(content_length, sizeof(content_length), "%d", total_len);
    esp_http_client_set_header(client, "Content-Length", content_length);

    ESP_LOGI(TAG, "Sending %d bytes to STT server...", total_len);

    esp_err_t err = esp_http_client_perform(client);
    free(upload_buf);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP status: %d", status);

    // Read response
    char resp_chunk[256];
    int total_read = 0;

    char *resp_buf = malloc(512);   // start small
    int resp_buf_size = 512;

    if (!resp_buf) {
        ESP_LOGE(TAG, "malloc failed");
        esp_http_client_cleanup(client);
        return NULL;
    }

    while (1) {
        int read_len = esp_http_client_read(client, resp_chunk, sizeof(resp_chunk) - 1);
        if (read_len <= 0) break;

        // Ensure null-terminated chunk
        resp_chunk[read_len] = 0;

        // Grow buffer if needed
        if (total_read + read_len + 1 > resp_buf_size) {
            resp_buf_size *= 2;
            char *new_buf = realloc(resp_buf, resp_buf_size);
            if (!new_buf) {
                ESP_LOGE(TAG, "realloc failed");
                free(resp_buf);
                esp_http_client_cleanup(client);
                return NULL;
            }
            resp_buf = new_buf;
        }

        memcpy(resp_buf + total_read, resp_chunk, read_len);
        total_read += read_len;
    }

    ESP_LOGI(TAG, "Raw body (%d bytes): '%.*s'", total_read, total_read, resp_buf);
    resp_buf[total_read] = 0;
    ESP_LOGI(TAG, "STT response: %s", resp_buf);
    esp_http_client_cleanup(client);
    return resp_buf;  // caller must free()
}