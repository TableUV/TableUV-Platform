/**
 * @file    app_supervisor.c
 * @author  Jianxiang (Jack) Xu
 * @date    15 Feb 2021
 * @brief   App level files
 *
 * This document will contains supervisor content
 */

#include "app_supervisor.h"

// Std. Lib
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// TableUV Lib
#include "common.h"
#include "app_slam.h"
#include "dev_avr_sensor.h"
#include "dev_avr_driver.h"
#include "dev_led.h"

// External Library

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef enum {
    APP_STATE_IDLE,
    APP_STATE_AUTONOMY,
    APP_STATE_AUTONOMY_ESTOPPED,
    //
    APP_STATE_COUNT,
    APP_STATE_UNKNOWN,
} app_state_E;

typedef enum {
    APP_FAULTS_CLEAR                = (0U),
    APP_FAULTS_LOW_BATTERY          = (1<<0U),
    APP_FAULTS_TOF_INIT             = (1<<1U),
    APP_FAULTS_IMU_INIT             = (1<<2U),
    // ...
    APP_FAULTS_MAX                  = (1<<20U),
    APP_FAULTS_INVALID_STATE        = (1<<30U),
} app_faults_E;

typedef struct{
    app_state_E current_state;
    uint32_t fault_flag; // Ex: (APP_FAULTS_TOF_INIT | APP_FAULTS_IMU_INIT);
    bool button_pressed;
    uint8_t avr_sensor_data;
} app_supervisor_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_supervisor_data_S supervisor_data;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_supervisor_init(void)
{
    memset(&supervisor_data, 0x00, sizeof(app_supervisor_data_S));
}


app_state_E getNextState(app_state_E state)
{
    app_state_E nextState = APP_STATE_UNKNOWN;
    // check all conditions, return the next state
    switch (state)
    {
        case (APP_STATE_IDLE):
            if (supervisor_data.button_pressed)
            {
                nextState =  APP_STATE_AUTONOMY;
            }
            break;

        case (APP_STATE_AUTONOMY):
            if (supervisor_data.button_pressed)
            {
                nextState =  APP_STATE_IDLE;
            }
            else if (supervisor_data.avr_sensor_data & DEV_AVR_ALL_SENSORS)
            {
                nextState = APP_STATE_AUTONOMY_ESTOPPED;
            }
            break;

        case (APP_STATE_AUTONOMY_ESTOPPED):
            if (supervisor_data.button_pressed)
            {
                nextState =  APP_STATE_IDLE;
            }
            else if ((supervisor_data.avr_sensor_data & DEV_AVR_ALL_SENSORS) == 0)
            {
                nextState = APP_STATE_AUTONOMY;
            }
            break;

        case (APP_STATE_COUNT):
            break;

        case (APP_STATE_UNKNOWN):
            break;
        default:
            // Do nothing
            supervisor_data.fault_flag |= APP_FAULTS_INVALID_STATE;
            break;
    }
    return nextState;
}

bool transitToNewState(app_state_E state)
{
    switch (state)
    {
        case (APP_STATE_IDLE):
            break;

        case (APP_STATE_AUTONOMY):
            dev_led_clear_leds();
            dev_led_green_set(true);
            break;

        case (APP_STATE_AUTONOMY_ESTOPPED):
            dev_led_clear_leds();
            dev_led_red_set(true);
            break;

        case (APP_STATE_COUNT):            
        case (APP_STATE_UNKNOWN):
        default:
            supervisor_data.fault_flag |= APP_FAULTS_INVALID_STATE;
            break;
    }
}

bool stateAction(app_state_E state)
{
    // whatever you should do in this state
}

void fetchState(void)
{
    // TODO: to be implemented
#if (FEATURE_PERIPHERALS)
    supervisor_data.button_pressed = dev_button_update_50ms();
#endif
#if (FEATURE_SENSOR_AVR)
    supervisor_data.avr_sensor_data = dev_avr_sensor_uart_get();
#endif

#if (FEATURE_SUPER_USE_PROFILED_MOTIONS | MOCK)
    // uint8_t frame=0U;
    // int8_t rmotor, lmotor;
    // frame = app_slam_getMotionVelocity(& lmotor, & rmotor, frame);
    // PRINTF("[MOTOR] R:%3d, L:%3d (f:%3d) \n", lmotor, rmotor, frame);
#endif

    // IR status
    // battery status
    // all the feeedbback
}

void app_supervisor_run20ms(void)
{
    const app_state_E current_state = supervisor_data.current_state;

    fetchState();

    app_state_E next_state = getNextState(current_state);

    if (next_state != current_state)
    {
        transitToNewState(current_state);
        supervisor_data.current_state = current_state;
    }

    stateAction(current_state);

}


