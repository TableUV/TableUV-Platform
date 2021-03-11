/**
 * @file    app_supervisor.c
 * @author  Jianxiang (Jack) Xu
 * @date    15 Feb 2021
 * @brief   App level files
 *
 * This document will contains supervisor content
 */

#include "app_supervisor.h"

// Std. Lib
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// TableUV Lib
#include "common.h"
#include "dev_avr_sensor.h"
#include "dev_avr_driver.h"

// External Library

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    uint8_t avr_sensor_data;
    robot_motion_mode_E avr_driver_cmd;
} app_supervisor_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_supervisor_data_S supervisor_data;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_supervisor_init(void)
{
    memset(&supervisor_data, 0x00, sizeof(app_supervisor_data_S));
}

void app_supervisor_run50ms(void)
{
    // TODO: to be implemented
#if (FEATURE_SENSOR_AVR)    
    supervisor_data.avr_sensor_data = dev_avr_sensor_uart_get();
    PRINTF("AVR Sensor data: %c%c%c%c%c%c%c%c\n", BYTE_TO_BINARY(supervisor_data.avr_sensor_data));
    if (supervisor_data.avr_sensor_data)
    {
        supervisor_data.avr_driver_cmd = ROBOT_MOTION_BREAK;
    }
    else
    {
        supervisor_data.avr_driver_cmd = ROBOT_MOTION_FW_COAST;
    }
#endif       
}