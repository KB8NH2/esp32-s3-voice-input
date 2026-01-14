#include "esp_lcd_touch.h"

esp_err_t esp_lcd_touch_read_data(esp_lcd_touch_handle_t tp)
{
    (void)tp;
    return ESP_OK;
}

esp_err_t esp_lcd_touch_get_data(esp_lcd_touch_handle_t tp, esp_lcd_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt)
{
    (void)tp; (void)data; (void)max_point_cnt;
    if (point_cnt) *point_cnt = 0;
    return ESP_OK;
}

bool esp_lcd_touch_get_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    (void)tp; (void)x; (void)y; (void)strength; (void)max_point_num;
    if (point_num) *point_num = 0;
    return false;
}

#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
esp_err_t esp_lcd_touch_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state)
{
    (void)tp; (void)n; (void)state;
    return ESP_ERR_NOT_SUPPORTED;
}
#endif

esp_err_t esp_lcd_touch_set_swap_xy(esp_lcd_touch_handle_t tp, bool swap)
{
    (void)tp; (void)swap; return ESP_OK;
}
esp_err_t esp_lcd_touch_get_swap_xy(esp_lcd_touch_handle_t tp, bool *swap)
{
    (void)tp; if (swap) *swap = false; return ESP_OK;
}
esp_err_t esp_lcd_touch_set_mirror_x(esp_lcd_touch_handle_t tp, bool mirror)
{
    (void)tp; (void)mirror; return ESP_OK;
}
esp_err_t esp_lcd_touch_get_mirror_x(esp_lcd_touch_handle_t tp, bool *mirror)
{
    (void)tp; if (mirror) *mirror = false; return ESP_OK;
}
esp_err_t esp_lcd_touch_set_mirror_y(esp_lcd_touch_handle_t tp, bool mirror)
{
    (void)tp; (void)mirror; return ESP_OK;
}
esp_err_t esp_lcd_touch_get_mirror_y(esp_lcd_touch_handle_t tp, bool *mirror)
{
    (void)tp; if (mirror) *mirror = false; return ESP_OK;
}

esp_err_t esp_lcd_touch_del(esp_lcd_touch_handle_t tp)
{
    (void)tp; return ESP_OK;
}

esp_err_t esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_handle_t tp, esp_lcd_touch_interrupt_callback_t callback)
{
    (void)tp; (void)callback; return ESP_OK;
}

esp_err_t esp_lcd_touch_register_interrupt_callback_with_data(esp_lcd_touch_handle_t tp, esp_lcd_touch_interrupt_callback_t callback, void *user_data)
{
    (void)tp; (void)callback; (void)user_data; return ESP_OK;
}

esp_err_t esp_lcd_touch_enter_sleep(esp_lcd_touch_handle_t tp)
{
    (void)tp; return ESP_OK;
}

esp_err_t esp_lcd_touch_exit_sleep(esp_lcd_touch_handle_t tp)
{
    (void)tp; return ESP_OK;
}
