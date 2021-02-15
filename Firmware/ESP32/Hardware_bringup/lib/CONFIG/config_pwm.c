/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : CONFIG_PWM.C
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: pwm config file for ESP32 
#####################################################################
*/

#include "pinconfig.h"
#include "driver/ledc.h"

#define PWM_CH_NUM        2
#define PWM_FREQUENCY     1000
#define PWM_RESOLUTION    LEDC_TIMER_13_BIT
#define PWM_SPEED_MODE    LEDC_HIGH_SPEED_MODE
#define PWM_TIMER_NUM     LEDC_TIMER_0



void pwm_steup(){
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION, // resolution of PWM duty
        .freq_hz = PWM_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = PWM_SPEED_MODE,           // timer mode
        .timer_num = PWM_TIMER_NUM,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
}