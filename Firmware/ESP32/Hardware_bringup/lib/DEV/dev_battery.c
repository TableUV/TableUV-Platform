/**
 * @file dev_battery.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_battery.h"

// Standard libraries 
#include <stdint.h>

// TableUV Lib
#include "../IO/io_ping_map.h"

// External Lib
#include "driver/gpio.h"
#include "driver/adc.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    float battery_voltage;
    int32_t battery_voltage_raw;
} dev_battery_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_battery_private_gpio_config(void);


///////////////////////////
///////   DATA     ////////
///////////////////////////
static dev_battery_data_S battery_data;

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
    io_conf.pin_bit_mask =  ((1ULL<< CHARGE_STATUS) | (1ULL<< BATTERY_VOLTAGE)); 
    gpio_config(&io_conf);
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_battery_init(void)
{
    dev_battery_private_gpio_config();
    adc2_config_channel_atten( ADC2_CHANNEL_2, ADC_ATTEN_DB_0 );
}

float dev_battery_convert_to_lipo_volts(int32_t raw_adc)
{
    return raw_adc * 0.8/4096 * (BATTERY_PULLUP_KOHMS + BATTERY_PULLDOWN_KOHMS) / BATTERY_PULLDOWN_KOHMS;
}

void dev_battery_update(void)
{
    esp_err_t r = adc2_get_raw( ADC2_CHANNEL_2, ADC_WIDTH_12Bit, &battery_data.battery_voltage_raw);
    if ( r == ESP_ERR_TIMEOUT ) 
    {
        printf("ADC2 used by Wi-Fi.\n");
    }

    battery_data.battery_voltage = dev_battery_convert_to_lipo_volts(battery_data.battery_voltage_raw);

}

float dev_battery_get(void)
{
    return battery_data.battery_voltage;
}

// Output: 0-4096
int32_t dev_battery_read_raw(void)
{
    int32_t raw_voltage = 0;

    esp_err_t r = adc2_get_raw( ADC2_CHANNEL_2, ADC_WIDTH_12Bit, &raw_voltage);
    if ( r == ESP_ERR_TIMEOUT ) 
    {
        printf("ADC2 used by Wi-Fi.\n");
    }

    return raw_voltage;
}