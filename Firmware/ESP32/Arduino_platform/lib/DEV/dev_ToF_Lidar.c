/**
 * @file dev_ToF_Lidar.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_ToF_Lidar.h"

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
static inline void dev_ToF_Lidar_private_gpio_config(void);


///////////////////////////
///////   DATA     ////////
///////////////////////////


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_ToF_Lidar_private_gpio_config()
{
    //IO mode select  
    gpio_pad_select_gpio(TOF_SHUT);
    gpio_pad_select_gpio(TOF_INT_1);
    gpio_pad_select_gpio(TOF_INT_2);
    gpio_pad_select_gpio(TOF_INT_3);

    gpio_config_t io_conf;

    //gpio config for output mode with open drain  
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask =  ((1ULL<< TOF_SHUT)); //TOF_SHUT has pullup on sensor 
    gpio_config(&io_conf);

    //gpio config for input & interrupt 
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask =  ((1ULL<< TOF_INT_1)|(1ULL<< TOF_INT_2 )|(1ULL<< TOF_INT_3)) ; 
    gpio_config(&io_conf);
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_ToF_Lidar_init(void)
{
    dev_ToF_Lidar_private_gpio_config();
}

