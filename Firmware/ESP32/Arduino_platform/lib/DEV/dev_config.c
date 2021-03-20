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
#if (FEATURE_AVR_DRIVER_ALL)    
    dev_avr_driver_init();
#endif    
#if (FEATURE_SENSOR_AVR)       
    dev_avr_sensor_init();
#endif    
#if (FEATURE_BATTERY)    
    dev_battery_init();
#endif    
#if (FEATURE_PERIPHERALS)    
    dev_led_init();
#endif    
#if (FEATURE_LIDAR)
    dev_ToF_Lidar_init();
#endif
#if (FEATURE_UV)
    dev_uv_init();
#endif    
#if (FEATURE_IMU)
    dev_imu_init();
#endif    
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
#if (FEATURE_AVR_DRIVER_ALL)
    dev_driver_avr_update100ms(); 
#endif   

}

void dev_run50ms(void)
{
 
#if (FEATURE_PERIPHERALS)
    dev_led_update();
#endif
}

void dev_run1000ms(void)
{
#if (FEATURE_BATTERY)    
    dev_battery_update();
#endif
}
