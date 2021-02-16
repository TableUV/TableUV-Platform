/**
 * @file main.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Main File
 *
 */

// Standard libraries 
#include <string.h>
#include <stdio.h>

// TableUV Lib
#include "dev_config.h"
#include "io_ping_map.h"

// SDK config 
#include "sdkconfig.h"

// for pwm setting 
#define PWM_CH_NUM        2
#define PWM_FREQUENCY     1000
#define PWM_RESOLUTION    LEDC_TIMER_13_BIT
#define PWM_SPEED_MODE    LEDC_HIGH_SPEED_MODE
#define PWM_TIMER_NUM     LEDC_TIMER_0

typedef enum {
    UVDRIVER_ARRAY_BOARD= 0, /*!< LEDC high speed speed_mode */
    UVDRIVER_SINGLE_BOARD,      /*!< LEDC low speed speed_mode */
} uv_driver_board_t;

void app_main()
{
    // device initialization
    dev_init();

    while(1) {

    }
}