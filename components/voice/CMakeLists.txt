file(GLOB_RECURSE component_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.c" "*.cpp")
idf_component_register(SRCS ${component_srcs}
                       INCLUDE_DIRS "."
                       REQUIRES esp_http_client nvs_flash mbedtls)
