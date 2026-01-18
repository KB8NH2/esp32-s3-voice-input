idf_component_register(
    SRCS "lvgl_driver.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lvgl_port lcd_driver hardware_driver
)
