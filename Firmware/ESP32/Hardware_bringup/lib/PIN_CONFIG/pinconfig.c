/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : PINCONFIG.C
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: pinconfig file for ESP32 
#####################################################################
*/

#include "pinconfig.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"

void pin_setup(){


    //IO mode select  
    gpio_pad_select_gpio(STATUS_RED_LED);
    gpio_pad_select_gpio(STATUS_GREEN_LED);
    
    
    
    //direction 
    gpio_set_direction(STATUS_RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(STATUS_GREEN_LED, GPIO_MODE_OUTPUT);



}