/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : CONFIG_GPIO.C
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: gpio config file for ESP32 
#####################################################################
*/

#include "pinconfig.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"



void gpio_setup(){

    //IO mode select  
    gpio_pad_select_gpio(STATUS_RED_LED);
    gpio_pad_select_gpio(STATUS_GREEN_LED);
    gpio_pad_select_gpio(SWITCH);
    gpio_pad_select_gpio(TOF_SHUT);
    gpio_pad_select_gpio(CHARGE_STATUS);
    gpio_pad_select_gpio(BATTERY_VOLTAGE);
    gpio_pad_select_gpio(FW_SHUTDOWN);
    gpio_pad_select_gpio(TOF_INT_1);
    gpio_pad_select_gpio(TOF_INT_2);
    gpio_pad_select_gpio(TOF_INT_3);

    gpio_config_t io_conf;

    //gpio config for output mode 
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =  ((1ULL<< STATUS_RED_LED) | (1ULL<<STATUS_GREEN_LED) | (1ULL<<FW_SHUTDOWN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //gpio config for output mode with open drain  
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask =  ((1ULL<< TOF_SHUT)); //TOF_SHUT has pullup on sensor 
    gpio_config(&io_conf);

    //gpio config for input 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =  ((1ULL<< CHARGE_STATUS)|(1ULL<< BATTERY_VOLTAGE)|(1ULL<< SWITCH)) ; 
    gpio_config(&io_conf);

    //gpio config for input & interrupt 
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask =  ((1ULL<< TOF_INT_1)|(1ULL<< TOF_INT_2 )|(1ULL<< TOF_INT_3)) ; 
    gpio_config(&io_conf);

}