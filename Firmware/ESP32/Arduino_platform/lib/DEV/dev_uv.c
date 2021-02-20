/**
 * @file dev_uv.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_uv.h"

// TableUV Lib
#include "../IO/io_ping_map.h"

// External Library
#include "driver/ledc.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define PWM_CH_NUM        2
#define PWM_FREQUENCY     1000
#define PWM_RESOLUTION    LEDC_TIMER_13_BIT
#define PWM_SPEED_MODE    LEDC_HIGH_SPEED_MODE
#define PWM_TIMER_NUM     LEDC_TIMER_0


/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_uv_private_gpio_config(void);


///////////////////////////
///////   DATA     ////////
///////////////////////////


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_uv_private_gpio_config(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION, // resolution of PWM duty
        .freq_hz = PWM_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = PWM_SPEED_MODE,           // timer mode
        .timer_num = PWM_TIMER_NUM,            // timer index
    };
    ledc_timer_config(&ledc_timer);
}


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_uv_init(void)
{
    dev_uv_private_gpio_config();
}

