#pragma once
#include "esp_err.h"

esp_err_t mqtt_start(const char *uri, const char *user, const char *pass);
void mqtt_publish(const char *topic, const char *payload);