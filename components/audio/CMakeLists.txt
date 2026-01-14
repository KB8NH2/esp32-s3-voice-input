message(STATUS "Skipping local components/audio to avoid name conflict with IDF audio component")
return()
file(GLOB_RECURSE component_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.c" "*.cpp")
idf_component_register(SRCS ${component_srcs}
                       INCLUDE_DIRS "."
                       REQUIRES esp_http_client nvs_flash mbedtls)
file(GLOB_RECURSE component_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.c" "*.cpp")
idf_component_register(SRCS ${component_srcs}
					   INCLUDE_DIRS "."
					   REQUIRES esp_http_client nvs_flash)
