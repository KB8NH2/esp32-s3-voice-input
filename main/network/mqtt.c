#include "mqtt.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "audio_driver.h"
#include "lvgl_example.h"   
#include "lvgl_driver.h"

static const char *TAG = "mqtt";
static esp_mqtt_client_handle_t client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, "home/esp32s3/mp3/play", event->topic_len) == 0) {
                Audio_Play_Music((const char*)event->data);
            }
            else if (strncmp(event->topic, "home/esp32s3/tts/play", event->topic_len) == 0) {
                Audio_Play_Music((const char*)event->data);
            }
            else if (strncmp(event->topic, "home/esp32s3/display/show", event->topic_len) == 0) {
                lvgl_show_message(event->data);
            }
            break;

        default:
            break;
    }
}

esp_err_t mqtt_start(const char *uri, const char *user, const char *pass)
{
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .uri = uri
            }
        },
        .credentials = {
            .username = user,
            .authentication = {
                .password = pass
            }
        },
        .session = {
            .keepalive = 30
        }
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    return ESP_OK;
}

void mqtt_publish(const char *topic, const char *payload)
{
    if (client) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
    }
}