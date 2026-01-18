file(GLOB_RECURSE component_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.c" "*.cpp")
idf_component_register(SRCS ${component_srcs}
					   INCLUDE_DIRS "."
				   REQUIRES esp_codec_dev esp_driver_i2s esp_driver_gpio driver espressif__led_strip hardware_driver tca9555_driver)
