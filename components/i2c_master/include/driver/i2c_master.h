/* Minimal shim for i2c_master API expected by project components. */
#ifndef I2C_MASTER_DRIVER_H
#define I2C_MASTER_DRIVER_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef void *i2c_master_bus_handle_t;
typedef void *i2c_master_dev_handle_t;

typedef struct {
	int i2c_port;
	int sda_io_num;
	int scl_io_num;
	int clk_source;
	int glitch_ignore_cnt;
	int intr_priority;
	int trans_queue_depth;
	struct { int enable_internal_pullup; } flags;
} i2c_master_bus_config_t;

typedef struct {
	uint8_t device_address;
	int scl_speed_hz;
} i2c_device_config_t;

esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *cfg, i2c_master_bus_handle_t *out_bus);
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *dev_cfg, i2c_master_dev_handle_t *out_dev);

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, int addr, int timeout_ms);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len, int timeout_ms);
esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev, uint8_t *data, size_t len, int timeout_ms);

#endif // I2C_MASTER_DRIVER_H
/* Minimal shim for i2c_master API expected by project components.
 * This provides lightweight stubs to allow the project to build.
 * Runtime behavior is intentionally minimal — replace with a full
 * implementation if you need real I2C interactions.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef void *i2c_master_bus_handle_t;
typedef void *i2c_master_dev_handle_t;

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, int addr, int timeout_ms);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len, int timeout_ms);
esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev, uint8_t *data, size_t len, int timeout_ms);
