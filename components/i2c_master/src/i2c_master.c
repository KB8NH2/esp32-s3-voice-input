#include "driver/i2c_master.h"
#include "esp_err.h"
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, int addr, int timeout_ms)
{
    (void)bus; (void)addr; (void)timeout_ms;
    // Shim: assume device is present so higher-level init can proceed
    return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len, int timeout_ms)
{
    (void)dev; (void)data; (void)len; (void)timeout_ms;
    return ESP_OK;
}

esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev, uint8_t *data, size_t len, int timeout_ms)
{
    (void)dev; (void)data; (void)len; (void)timeout_ms;
    return ESP_OK;
}

// Lightweight stubs for bus/device management to satisfy callers in audio driver.
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *cfg, i2c_master_bus_handle_t *out_bus)
{
    (void)cfg;
    if (out_bus) *out_bus = (i2c_master_bus_handle_t)1; // non-NULL dummy handle
    return ESP_OK;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *dev_cfg, i2c_master_dev_handle_t *out_dev)
{
    (void)bus; (void)dev_cfg;
    if (out_dev) *out_dev = (i2c_master_dev_handle_t)1; // non-NULL dummy device
    return ESP_OK;
}
