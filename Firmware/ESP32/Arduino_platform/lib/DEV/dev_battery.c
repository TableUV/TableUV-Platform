/**
 * @file dev_battery.c
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_battery.h"

#define sei()
#define cli()

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct {
    float battery_voltage;              // Volts
    int32_t battery_voltage_raw;        // 0 - 4096
    charger_ic_status_E charger_status;
} dev_battery_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_battery_private_gpio_config(void);


///////////////////////////
///////   DATA     ////////
///////////////////////////
static dev_battery_data_S battery_data;
static volatile uint8_t edge_count = 0;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static void IRAM_ATTR charger_fault_isr_handler(void* arg)
{
    cli();
    edge_count++;
    sei();
}

static inline void dev_battery_private_gpio_config(void)
{
    //IO mode select  
    gpio_pad_select_gpio(CHARGE_STATUS);
    gpio_pad_select_gpio(BATTERY_VOLTAGE);
    
    gpio_config_t io_conf;
    //gpio config for input 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =  (1ULL<< BATTERY_VOLTAGE); 
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL<< CHARGE_STATUS);
    io_conf.intr_type = GPIO_INTR_POSEDGE;

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CHARGE_STATUS, charger_fault_isr_handler, (void*)CHARGE_STATUS);
    gpio_intr_enable(CHARGE_STATUS);
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
    return raw_adc * 0.8/4096 * (DEV_BATTERY_PULLUP_KOHMS + DEV_BATTERY_PULLDOWN_KOHMS) / DEV_BATTERY_PULLDOWN_KOHMS;
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

void dev_charger_status_update(void)
{
    if(gpio_get_level(CHARGE_STATUS))
    {
        battery_data.charger_status = CHARGE_COMPLETE_SLEEP;
    }
    else
    {
        battery_data.charger_status = CHARGING;
    }

    cli();
    if(edge_count >= DEV_BATTERY_CHARGE_STAT_FREQ_HZ/DEV_BATTERY_CHARGE_FAULT_FREQ_HZ)
    {
        battery_data.charger_status = CHARGER_FAULT;
    }
    edge_count = 0;
    sei();
}

charger_ic_status_E dev_charger_status_get(void)
{
    return battery_data.charger_status;
}

charger_ic_status_E dev_charger_status_read(void)
{
    dev_charger_status_update();
    return battery_data.charger_status;
}
