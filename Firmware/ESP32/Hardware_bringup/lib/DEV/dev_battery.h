/**
 * @file dev_battery.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */


#ifndef DEV_BATTERY_H
#define DEV_BATTERY_H

#include <stdint.h>

#define BATTERY_PULLUP_KOHMS    249
#define BATTERY_PULLDOWN_KOHMS  16.9

void dev_battery_init(void);
float dev_battery_convert_to_lipo_volts(int32_t raw_adc);
void dev_battery_update(void);
float dev_battery_get(void);
int32_t dev_battery_read_raw(void);


#endif //DEV_BATTERY_H