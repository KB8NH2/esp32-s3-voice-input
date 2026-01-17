#include "conversation_api.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "conversation";

static conversation_reply_callback_t s_conv_cb = NULL;
static char s_conv_url[128];

// Expecting Home Assistant Conversation API endpoint
// e.g. http://homeassistant.local:8123/api/conversation/process
// with token-based auth

// TODO: set your HA URL and token
//#define HA_TOKEN "YOUR_LONG_LIVED_ACCESS_TOKEN"

void conversation_init(const char *url)
{
    strncpy(s_conv_url, url, sizeof(s_conv_url));
    s_conv_url[sizeof(s_conv_url) - 1] = '\0';
    ESP_LOGD(TAG, "Conversation API URL: %s", s_conv_url);
}

void conversation_set_callback(conversation_reply_callback_t cb)
{
    s_conv_cb = cb;
}

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char resp_buf[2048];
    static size_t resp_len = 0;

    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (evt->data_len + resp_len < sizeof(resp_buf)) {
            memcpy(resp_buf + resp_len, evt->data, evt->data_len);
            resp_len += evt->data_len;
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        resp_buf[resp_len] = '\0';
        ESP_LOGD("conversation", "Conversation server response: %s", resp_buf);

        // TODO: parse JSON and extract assistant reply
        const char *reply = resp_buf;

        if (s_conv_cb) {
            s_conv_cb(reply);
        }

        resp_len = 0;
        break;
    default:
        break;
    }
    return ESP_OK;
}

void conversation_send(const char *user_text)
{
    ESP_LOGD(TAG, "Sending to Conversation Server: \"%s\"", user_text);

    esp_http_client_config_t cfg = {
        .url = s_conv_url,
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .timeout_ms = 15000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);

//    char auth_header[256];
//    snprintf(auth_header, sizeof(auth_header), "Bearer %s", HA_TOKEN);
//    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    char clean[480];
    strncpy(clean, user_text, sizeof(clean) - 1);
    clean[sizeof(clean) - 1] = '\0';
    size_t clean_len = strlen(clean);
    while (clean_len > 0 && (clean[clean_len - 1] == '\n' || clean[clean_len - 1] == '\r')) {
        clean[--clean_len] = '\0';
    }

    char body[512];
    snprintf(body, sizeof(body),
             "{\"input\": \"%s\"}", clean);
    ESP_LOGD(TAG, "Sending to Conversation Server (full body): %s", body);

    esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Conversation request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}