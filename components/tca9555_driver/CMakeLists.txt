idf_component_register(
    SRCS "tca9555_driver.c"
    INCLUDE_DIRS "."
    REQUIRES espressif__esp_io_expander_tca95xx_16bit hardware_driver audio_play_driver
)
