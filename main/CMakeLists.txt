# Core application sources
set(SOURCES
    "./main.c"

    # Audio input/output
    "./speech_det_driver/mic_speech.c"
    "./audio_play_driver/audio_driver.c"

    # VAD
    "./audio/vad.c"

    # Networking + Wyoming clients
    "./network/wifi.c"
    "./network/stt.c"
    "./network/conversation_api.c"
    "./network/piper_tts.c"
)

# Include directories
set(INCLUDE_DIRS
    "."
    "./speech_det_driver"
    "./audio_play_driver"
    "./audio"
    "./network"
    "../managed_components"
)

# Register component
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES esp_codec_dev
    REQUIRES esp_driver_gpio
    REQUIRES esp_driver_i2s
    REQUIRES esp_wifi
    REQUIRES esp_timer
    REQUIRES espressif__gmf_core
    REQUIRES esp_http_client
    REQUIRES nvs_flash
)