/**
 * @file dev_battery.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */

// Standard libraries 
#include <stdint.h>

// TableUV Lib
#include "../IO/io_ping_map.h"

// External Lib
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Defines
#ifndef DEV_BATTERY_H
#define DEV_BATTERY_H

#define BATTERY_PULLUP_KOHMS    249
#define BATTERY_PULLDOWN_KOHMS  16.9

// Function Prototypes
void dev_battery_init(void);
float dev_battery_convert_to_lipo_volts(int32_t raw_adc);
void dev_battery_update(void);
float dev_battery_get(void);
int32_t dev_battery_read_raw(void);
void dev_charger_status_update(void);
int8_t dev_charger_status_get(void);
int8_t dev_charger_status_read(void);


#endif //DEV_BATTERY_H