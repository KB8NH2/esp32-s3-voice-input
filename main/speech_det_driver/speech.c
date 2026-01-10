#include "speech.h"
#include "audio_driver.h"
#include "vad.h"
#include "mic_speech.h"
#include "esp_log.h"

#define TAG "Speech"
// -----------------------------
// Public entry point
// -----------------------------
void Speech_Init(void)
{
    ESP_LOGI("Speech", "Initializing speech subsystem...");

    esp_err_t err = audio_driver_init(&i2s_tx_handle, &i2s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE("Speech", "audio_driver_init failed: %s", esp_err_to_name(err));
        return;
    }

    if (i2s_rx_handle == NULL) {
        ESP_LOGE("Speech", "ERROR: i2s_rx_handle is NULL after audio_driver_init()");
        return;
    }

    vad_reset_state();      // or vad_init()

    mic_speech_start();

    ESP_LOGI("Speech", "Speech subsystem initialized");
}