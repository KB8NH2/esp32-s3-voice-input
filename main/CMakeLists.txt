# Core application sources
set(SOURCES
    "./main.c"
    # Networking + Wyoming clients
    "./network/wifi.c"
    "./network/stt.c"
    "./network/conversation_api.c"
    "./network/piper_tts.c"
)

# Include directories
set(INCLUDE_DIRS
    "."
    "./network"
    "../components"
    "../managed_components"
)

# Register component
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES esp_codec_dev
    REQUIRES esp_driver_gpio
    REQUIRES esp_driver_i2s
    REQUIRES speech_det_driver
    REQUIRES esp_wifi
    REQUIRES esp_timer
    REQUIRES espressif__gmf_core
    REQUIRES esp_http_client
    REQUIRES nvs_flash
)