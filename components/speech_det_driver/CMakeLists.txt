file(GLOB_RECURSE component_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.c" "*.cpp")

idf_component_register(SRCS ${component_srcs}
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2s esp_codec_dev voice audio_play_driver i2c_master
                       PRIV_REQUIRES esp_timer)

