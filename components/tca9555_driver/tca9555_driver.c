#include "tca9555_driver.h"
#include "audio_driver.h"
#include "esp_log.h"
#include "bsp_board.h"
#include "driver/i2c_master.h"


esp_io_expander_handle_t io_expander = NULL;


void tca9555_driver_init(void)
{

    i2c_master_bus_handle_t i2c_bus = audio_driver_get_i2c_bus();
    if (!i2c_bus) {
        printf("tca9555_driver_init: i2c_bus is NULL!\r\n");
        return;
    }
    
    esp_err_t ret = esp_io_expander_new_i2c_tca95xx_16bit(i2c_bus, ESP_IO_EXPANDER_I2C_TCA9555_ADDRESS_000, &io_expander);
    if (ret != ESP_OK) {
        printf("tca9555_driver_init: esp_io_expander_new failed! ret=0x%x\r\n", (unsigned)ret);
        return;
    }
    printf("tca9555_driver_init: io_expander created successfully at %p\r\n", io_expander);

   /* Test output level function */
   ret = esp_io_expander_set_dir(io_expander, (IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_5 | IO_EXPANDER_PIN_NUM_6 | IO_EXPANDER_PIN_NUM_8), IO_EXPANDER_OUTPUT);
   if(ret != ESP_OK){
    printf("EXIO Mode setting failure (OUTPUT pins)!! ret=0x%x\r\n", (unsigned)ret);
   }
   
   ret = esp_io_expander_set_dir(io_expander, (IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_9 | IO_EXPANDER_PIN_NUM_10 | IO_EXPANDER_PIN_NUM_11), IO_EXPANDER_INPUT);   
   if(ret != ESP_OK){
    printf("EXIO Mode setting failure (INPUT pins)!! ret=0x%x\r\n", (unsigned)ret);
   }
   
   // Enable weak pull-ups on input pins by writing to output registers directly
   // TCA9555 registers: 0x02=Output Port 0, 0x03=Output Port 1
   // Input pins: pin2=P0.2, pin9=P1.1, pin10=P1.2, pin11=P1.3
   // We need to set bits high in output registers for input pins to enable pull-ups
   i2c_device_config_t tca_cfg = {
       .device_address = 0x20,
       .scl_speed_hz = 100000,
   };
   i2c_master_dev_handle_t tca_dev = NULL;
   ret = i2c_master_bus_add_device(i2c_bus, &tca_cfg, &tca_dev);
   if (ret == ESP_OK) {
       // Write to Output Port 0 (0x02): set bit 2 high for pin2 pull-up
       uint8_t port0_data[2] = {0x02, 0x04};  // bit 2 = 0x04
       i2c_master_transmit(tca_dev, port0_data, 2, 100);
       
       // Write to Output Port 1 (0x03): set bits 1,2,3 high for pin9,10,11 pull-ups
       uint8_t port1_data[2] = {0x03, 0x0E};  // bits 1,2,3 = 0x0E
       i2c_master_transmit(tca_dev, port1_data, 2, 100);
       
       i2c_master_bus_rm_device(tca_dev);
       printf("TCA9555 pull-ups enabled via direct register write\r\n");
   } else {
       printf("Failed to add temp I2C device for pull-up config: ret=0x%x\r\n", (unsigned)ret);
   }
   
   printf("tca9555_driver_init: initialization complete\r\n");
}


/********************************************************** Set the EXIO output status **********************************************************/  
void Set_EXIO(uint32_t Pin,uint8_t State)                  // Sets the level state of the Pin without affecting the other pins(PIN：1~8)
{
    esp_err_t ret;
    ret = esp_io_expander_set_level(io_expander, Pin, State);      
    if(ret != ESP_OK){
        printf("EXIO level setting failure!!\r\n");
    }

}
/********************************************************** Read EXIO status **********************************************************/       
bool Read_EXIO(uint32_t Pin)                            // Read the level of the TCA9555PWR Pin
{
    if (!io_expander) {
        printf("Read_EXIO: io_expander is NULL! Pin=%d\r\n", (int)Pin);
        return false;
    }
    
    esp_err_t ret;
    uint32_t input_level_mask = 0;            
    ret = esp_io_expander_get_level(io_expander, Pin, &input_level_mask);      
    if(ret != ESP_OK){
        printf("EXIO level reading failure!! Pin=%d ret=0x%x\r\n", (int)Pin, (unsigned)ret);
        return false;
    }          
    bool bitStatus = input_level_mask & Pin;                             
    return bitStatus;                                                              
}