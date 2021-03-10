/**
 * @file dev_led.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_led.h"

// TableUV Lib
#include "../IO/io_ping_map.h"

// External Lib
#include "driver/gpio.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    bool button_pressed;
    bool green_led_on;
    bool red_led_on;
    bool orange_led_on;
} dev_led_peripherals_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_led_private_gpio_config(void);

///////////////////////////
///////   DATA     ////////
///////////////////////////
static dev_led_peripherals_data_S peripheral_data = {
    .button_pressed = false,
    .green_led_on = false,
    .red_led_on = false,
    .orange_led_on = false
};


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_led_private_gpio_config(void)
{
    gpio_pad_select_gpio(STATUS_RED_LED);
    gpio_pad_select_gpio(STATUS_GREEN_LED);
    gpio_pad_select_gpio(FW_SHUTDOWN);
    gpio_pad_select_gpio(BUTTON);

    //gpio config for output mode 
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =  ((1ULL<< STATUS_RED_LED) | (1ULL<<STATUS_GREEN_LED) | (1ULL<<FW_SHUTDOWN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON);
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_led_init(void)
{
    dev_led_private_gpio_config();
}

void dev_button_update(void)
{
    peripheral_data.button_pressed = !(GPIO_INPUT_GET(BUTTON));
}

bool dev_button_get(void)
{
    return peripheral_data.button_pressed;
}

void dev_led_update(void)
{
    if(peripheral_data.green_led_on)
    {
        GPIO_OUTPUT_SET(STATUS_GREEN_LED, 1);
    }
    else
    {
        GPIO_OUTPUT_SET(STATUS_GREEN_LED, 0);
    }
    if(peripheral_data.red_led_on)
    {
        GPIO_OUTPUT_SET(STATUS_RED_LED, 1);
    }
    else
    {
        GPIO_OUTPUT_SET(STATUS_RED_LED, 0);
    }
    if(peripheral_data.orange_led_on)
    {
        // Send I2C to Avr
    }
    else
    {
        // Send I2C to Avr
    }        
}

void dev_led_green_set(bool led_on)
{
    peripheral_data.green_led_on = led_on;
}

void dev_led_red_set(bool led_on)
{
    peripheral_data.red_led_on = led_on;
}

void dev_led_orange_set(bool led_on)
{
    peripheral_data.orange_led_on = led_on;
}
