/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : CONFIG_I2C.C
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: i2c config file for ESP32 
#####################################################################
*/

#include "pinconfig.h"
#include "driver/i2c.h"


#define I2C_MASTER_FREQUENCY_NORMAL    100000
#define I2C_MASTER_FREQUENCY_FAST      400000
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

void i2c_setup(){

    // i2c_port_t i2c_master_port = 0;
    // i2c_config_t conf = {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = TOF_I2C_SDA,         
    //     .sda_pullup_en = GPIO_PULLUP_DISABLE,
    //     .scl_io_num = TOF_I2C_SCL,       
    //     .scl_pullup_en = GPIO_PULLUP_DISABLE,
    //     .master.clk_speed = I2C_MASTER_FREQUENCY_NORMAL,  // 
    //     // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    // };
    // esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    // if (err != ESP_OK) {
    //     return err;
    // }
    // i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);


}