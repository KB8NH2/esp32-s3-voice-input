#include "lcd_driver.h"
#include "esp_log.h"

static const char *TAG = "lcd_driver";

esp_lcd_panel_io_handle_t lcd_io;
esp_lcd_panel_handle_t lcd_panel;

esp_err_t lcd_driver_init(void)
{
    ESP_LOGI(TAG, "lcd_driver_init (stub)");
    return ESP_OK;
#include "lcd_driver.h"
#include "esp_log.h"

static const char *TAG = "lcd_driver";

esp_lcd_panel_io_handle_t lcd_io;
esp_lcd_panel_handle_t lcd_panel;

esp_err_t lcd_driver_init(void)
{
    ESP_LOGI(TAG, "lcd_driver_init (stub)");
    return ESP_OK;
}

void Set_Backlight(uint8_t Light)
{
    (void)Light;
}

uint8_t Read_Backlight_value(void)
{
    return 0;
}
}
