/**
 * @file dev_battery.h
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */

#ifndef DEV_BATTERY_H
#define DEV_BATTERY_H
# ifdef __cplusplus
extern "C"{
# endif 

// Standard libraries 
#include <stdint.h>

// TableUV Lib
#include "../IO/io_ping_map.h"

// External Lib
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEV_BATTERY_PULLUP_KOHMS                249
#define DEV_BATTERY_PULLDOWN_KOHMS              16.9
#define DEV_BATTERY_CHARGE_STAT_FREQ_HZ         10
#define DEV_BATTERY_CHARGE_FAULT_FREQ_HZ        2

typedef enum {
    CHARGING,                   // STAT Pin Low
    CHARGE_COMPLETE_SLEEP,      // STAT Pin HIGH
    CHARGER_FAULT               // STAT Pin Blinkings
} charger_ic_status_E;

// Function Prototypes
void dev_battery_init(void);
float dev_battery_convert_to_lipo_volts(int32_t raw_adc);
void dev_battery_update(void);
float dev_battery_get(void);
int32_t dev_battery_read_raw(void);
void dev_charger_status_update(void);
charger_ic_status_E dev_charger_status_get(void);
charger_ic_status_E dev_charger_status_read(void);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_BATTERY_H