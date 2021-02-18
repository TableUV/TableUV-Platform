/**
 * @file dev_battery.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_battery.h"

// TableUV Lib
#include "../IO/io_ping_map.h"

// External Lib
#include "driver/gpio.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////


/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_battery_private_gpio_config(void);


///////////////////////////
///////   DATA     ////////
///////////////////////////


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_battery_private_gpio_config(void)
{
    //IO mode select  
    gpio_pad_select_gpio(CHARGE_STATUS);
    gpio_pad_select_gpio(BATTERY_VOLTAGE);
    
    gpio_config_t io_conf;
    //gpio config for input 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =  (1ULL<< CHARGE_STATUS); 
    gpio_config(&io_conf);

    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pin_bit_mask = (1ULL<< BATTERY_VOLTAGE);
    gpio_config(&io_conf);
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_battery_init(void)
{
    dev_battery_private_gpio_config();
}

