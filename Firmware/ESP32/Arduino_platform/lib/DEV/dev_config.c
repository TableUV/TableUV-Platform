/**
 * @file dev_config.c
	 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_config.h"

// TableUV Lib
#include "dev_avr_driver.h"
#include "dev_avr_sensor.h"
#include "dev_battery.h"
#include "dev_config.h"
#include "dev_led.h"
#include "dev_ToF_Lidar.h"
#include "dev_uv.h"
#include "dev_imu.h"
#include "../../include/common.h"


/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////


/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////


///////////////////////////
///////   DATA     ////////
///////////////////////////


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_init(void)
{
    // sub-device  initialization
    dev_avr_driver_init();
    dev_avr_sensor_init();
    dev_battery_init();
    dev_led_init();
#if (FEATURE_LIDAR)
    dev_ToF_Lidar_init();
#endif
    //dev_uv_init();
    //dev_imu_init();
}

void dev_run20ms(void)
{
    // Do  nothing
#if (FEATURE_LIDAR)
    dev_ToF_Lidar_update20ms();
#endif
#if (FEATURE_SENSOR_AVR)
    dev_avr_sensor_uart_update();
#endif
}

void dev_run100ms(void)
{
#if (FEATURE_AVR_DRIVER_ALL)
    dev_driver_avr_update100ms(); 
#endif
}

void dev_run1000ms(void)
{
}
