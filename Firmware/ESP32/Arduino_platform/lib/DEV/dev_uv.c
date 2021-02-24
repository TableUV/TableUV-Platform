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
#include "driver/dac.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define PWM_FREQUENCY       (1000)
#define PWM_RESOLUTION      LEDC_TIMER_13_BIT
#define PWM_SPEED_MODE      LEDC_HIGH_SPEED_MODE
#define PWM_TIMER_NUM       LEDC_TIMER_0

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_uv_private_gpio_config(void);
typedef struct {
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel[DEV_UV_LED_COUNT];
} dev_uv_data_S;

///////////////////////////
///////   DATA     ////////
///////////////////////////
dev_uv_data_S uv_data = {
    {
        .duty_resolution    = PWM_RESOLUTION,       // resolution of PWM duty
        .freq_hz            = PWM_FREQUENCY,        // frequency of PWM signal
        .speed_mode         = PWM_SPEED_MODE,       // timer mode
        .timer_num          = PWM_TIMER_NUM,        // timer index
    },
    {
        {
            .channel        = LEDC_CHANNEL_0,
            .duty           = 0,
            .gpio_num       = LED_ROW_MODULATE,
            .speed_mode     = PWM_SPEED_MODE,
            .hpoint         = 0,
            .timer_sel      = PWM_TIMER_NUM
        },
        {
            .channel        = LEDC_CHANNEL_1,
            .duty           = 0,
            .gpio_num       = LED_SIDE_MODULATE,
            .speed_mode     = PWM_SPEED_MODE,
            .hpoint         = 0,
            .timer_sel      = PWM_TIMER_NUM
        }
    }
};


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_uv_private_gpio_config(void)
{
    ledc_timer_config(&uv_data.ledc_timer);

    for (int ch = 0; ch < DEV_UV_LED_COUNT; ch++)
    {
        ledc_channel_config(&uv_data.ledc_channel[ch]);
    }
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_uv_init(void)
{
    dev_uv_private_gpio_config();    
}

void dev_uv_set_both(int pwm_duty, uint8_t dac_duty)
{
    ledc_channel_config_t* channel = &(uv_data.ledc_channel);
    for (int ch = 0; ch < DEV_UV_LED_COUNT; ch++)
    {
        dac_output_voltage(ESP_DAC, dac_duty);
        ledc_set_duty(channel[ch].speed_mode, channel[ch].channel, pwm_duty);
        ledc_update_duty(channel[ch].speed_mode, channel[ch].channel);
    }
}

void dev_uv_set_row(int pwm_duty, uint8_t dac_duty)
{
    ledc_channel_config_t* row = &(uv_data.ledc_channel[DEV_UV_LED_ROW]);
    dac_output_voltage(ESP_DAC, dac_duty);
    ledc_set_duty(row->speed_mode, row->channel, pwm_duty);
    ledc_update_duty(row->speed_mode, row->channel);
}

void dev_uv_set_side(int pwm_duty, uint8_t dac_duty)
{
    ledc_channel_config_t* side = &(uv_data.ledc_channel[DEV_UV_LED_SIDE]);
    dac_output_voltage(ESP_DAC, dac_duty);
    ledc_set_duty(side->speed_mode, side->channel, pwm_duty);
    ledc_update_duty(side->speed_mode, side->channel);
}

void dev_uv_stop() 
{
    ledc_channel_config_t* channel = &(uv_data.ledc_channel);
    for (int ch = 0; ch < DEV_UV_LED_COUNT; ch++)
    {
        ledc_stop(channel[ch].speed_mode, channel[ch].channel, 0);
    }
}