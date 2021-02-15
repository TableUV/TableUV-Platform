/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : CONFIG.C
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: config file for ESP32 
#####################################################################
*/

#include "config.h"

#include "config_gpio.c"
#include "config_i2c.c"
#include "config_dac.c"
#include "config_pwm.c"

void init_setup(){

    gpio_setup();
    dac_setup();
    pwm_steup();
}
