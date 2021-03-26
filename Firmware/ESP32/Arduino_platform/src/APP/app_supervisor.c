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
#include "dev_battery.h"
#include "app_slam.h"
#include "dev_uv.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define BATTERY_STATUS              (10.5F)
#define UV_PWM                      (500)
#define UV_DAC                      (64)

typedef enum {
    APP_STATE_IDLE,
    APP_STATE_AUTONOMY,
    APP_STATE_AUTONOMY_ESTOPPED,
    APP_STATE_FAULT,
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

#if (FEATURE_SUPER_USE_HARDCODE_CHORE)
#define CHOREOGRAPHY_BASE_TICK_20MS    (10U) // base ticK 10*20ms

typedef enum{
    APP_CHOREOGRAPHY_INIT,
    APP_CHOREOGRAPHY_STEP_1,
    APP_CHOREOGRAPHY_STEP_2,
    APP_CHOREOGRAPHY_STEP_3,
    APP_CHOREOGRAPHY_STEP_4,
    APP_CHOREOGRAPHY_STEP_5,
    APP_CHOREOGRAPHY_STEP_6,
    APP_CHOREOGRAPHY_STEP_7,
    APP_CHOREOGRAPHY_STEP_8,
    APP_CHOREOGRAPHY_STEP_9,
    APP_CHOREOGRAPHY_STEP_COUNT,
    APP_CHOREOGRAPHY_STEP_UNKNOWN,
} app_choreography_E;
#endif // (FEATURE_SUPER_USE_HARDCODE_CHORE)

typedef struct{
    app_state_E         current_state;
    uint32_t            fault_flag; // Ex: (APP_FAULTS_TOF_INIT | APP_FAULTS_IMU_INIT);
    bool                button_pressed;
    uint8_t             avr_sensor_data;
    float               battery_voltage;
    uint16_t            app_slam_EFlag;
#if (FEATURE_SUPER_USE_HARDCODE_CHORE)
    uint32_t                estop_chorography_tick_20ms;
    app_choreography_E      estop_choreography_wip;
    const motor_pwm_duty_E  estop_choreography_sequence_l[APP_CHOREOGRAPHY_STEP_COUNT];
    const motor_pwm_duty_E  estop_choreography_sequence_r[APP_CHOREOGRAPHY_STEP_COUNT];
    const motor_pwm_duty_E  estop_choreography_sequence_mode[APP_CHOREOGRAPHY_STEP_COUNT];
#endif // (FEATURE_SUPER_USE_HARDCODE_CHORE)
} app_supervisor_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static bool app_supervisor_private_exitCurrentState(app_state_E state);
static bool app_supervisor_private_stateAction(app_state_E state);
static void app_supervisor_private_fetchState(void);
static bool app_supervisor_private_transitToNewState(app_state_E state);
static app_state_E app_supervisor_private_getNextState(app_state_E state);

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_supervisor_data_S supervisor_data = {
    .current_state   = APP_STATE_IDLE,
    .fault_flag      = 0,
    .button_pressed  = false,
    .avr_sensor_data = 0,
    .battery_voltage = 12.6,
    .app_slam_EFlag  = APP_SLAM_TOF_DANGER_ZONE_FLAG_NULL,
#if (FEATURE_SUPER_USE_HARDCODE_CHORE)
    .estop_chorography_tick_20ms = 0U,
    .estop_choreography_wip      = APP_CHOREOGRAPHY_STEP_UNKNOWN,
    .estop_choreography_sequence_l = {
        [APP_CHOREOGRAPHY_INIT  ] = MOTOR_PWM_DUTY_0_PERCENT,
        [APP_CHOREOGRAPHY_STEP_1] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_2] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_3] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_4] = MOTOR_PWM_DUTY_0_PERCENT,
        [APP_CHOREOGRAPHY_STEP_5] = MOTOR_PWM_DUTY_0_PERCENT,
        [APP_CHOREOGRAPHY_STEP_6] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_7] = MOTOR_PWM_DUTY_40_PERCENT,
        [APP_CHOREOGRAPHY_STEP_8] = MOTOR_PWM_DUTY_40_PERCENT,
        [APP_CHOREOGRAPHY_STEP_9] = MOTOR_PWM_DUTY_0_PERCENT,
    },
    .estop_choreography_sequence_r = {
        [APP_CHOREOGRAPHY_INIT  ] = MOTOR_PWM_DUTY_0_PERCENT,
        [APP_CHOREOGRAPHY_STEP_1] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_2] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_3] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_4] = MOTOR_PWM_DUTY_0_PERCENT,
        [APP_CHOREOGRAPHY_STEP_5] = MOTOR_PWM_DUTY_0_PERCENT,
        [APP_CHOREOGRAPHY_STEP_6] = MOTOR_PWM_DUTY_30_PERCENT,
        [APP_CHOREOGRAPHY_STEP_7] = MOTOR_PWM_DUTY_40_PERCENT,
        [APP_CHOREOGRAPHY_STEP_8] = MOTOR_PWM_DUTY_40_PERCENT,
        [APP_CHOREOGRAPHY_STEP_9] = MOTOR_PWM_DUTY_0_PERCENT,
    },
    .estop_choreography_sequence_mode = {
        [APP_CHOREOGRAPHY_INIT  ] = ROBOT_MOTION_BREAK,
        [APP_CHOREOGRAPHY_STEP_1] = ROBOT_MOTION_REV_COAST,
        [APP_CHOREOGRAPHY_STEP_2] = ROBOT_MOTION_REV_COAST,
        [APP_CHOREOGRAPHY_STEP_3] = ROBOT_MOTION_BREAK,
        [APP_CHOREOGRAPHY_STEP_4] = ROBOT_MOTION_BREAK,
        [APP_CHOREOGRAPHY_STEP_5] = ROBOT_MOTION_BREAK,
        [APP_CHOREOGRAPHY_STEP_6] = ROBOT_MOTION_CW_ROTATION,
        [APP_CHOREOGRAPHY_STEP_7] = ROBOT_MOTION_CW_ROTATION,
        [APP_CHOREOGRAPHY_STEP_8] = ROBOT_MOTION_CW_ROTATION,
        [APP_CHOREOGRAPHY_STEP_9] = ROBOT_MOTION_BREAK,
    },
#endif // (FEATURE_SUPER_USE_HARDCODE_CHORE)
};

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////

static app_state_E app_supervisor_private_getNextState(app_state_E state)
{
    app_state_E nextState = state;
    // check all conditions, return the next state
    switch (state)
    {
        case (APP_STATE_IDLE):
            if (supervisor_data.button_pressed)
            {
                // TODO: Starting on powerup problem
                nextState =  APP_STATE_AUTONOMY;
            }
            break;

        case (APP_STATE_AUTONOMY):
            if (supervisor_data.button_pressed)
            {
                nextState =  APP_STATE_IDLE; // Highest priority
            }
            else if (supervisor_data.battery_voltage < BATTERY_STATUS) // check last, for the least probable state
            {
                supervisor_data.fault_flag |= APP_FAULTS_LOW_BATTERY;
                nextState = APP_STATE_FAULT;
            }
            else if (supervisor_data.avr_sensor_data & DEV_AVR_ALL_SENSORS)
            {
                nextState = APP_STATE_AUTONOMY_ESTOPPED;
            }
            else if (supervisor_data.app_slam_EFlag != APP_SLAM_TOF_DANGER_ZONE_FLAG_NULL)
            {
                nextState = APP_STATE_AUTONOMY_ESTOPPED;
            }
            break;

        case (APP_STATE_AUTONOMY_ESTOPPED):
            if (supervisor_data.button_pressed)
            {
                nextState =  APP_STATE_IDLE; // Highest priority
            }
            else if (supervisor_data.battery_voltage < BATTERY_STATUS)
            {
                supervisor_data.fault_flag |= APP_FAULTS_LOW_BATTERY;
                nextState = APP_STATE_FAULT;
            }    
#if (FEATURE_SUPER_USE_HARDCODE_CHORE)
            else if (supervisor_data.estop_choreography_wip < APP_CHOREOGRAPHY_STEP_COUNT)
            {
                // choreography still in progress
#   if (DEBUG_FPRINT_FEATURE_CHOREOGRAPHY)
                PRINTF("[ CHOREOGRAPHY ]: %d \n", supervisor_data.estop_choreography_wip);
#   endif//(DEBUG_FPRINT_FEATURE_CHOREOGRAPHY)
            }
#endif // (FEATURE_SUPER_USE_HARDCODE_CHORE)
            else if ((supervisor_data.avr_sensor_data & DEV_AVR_ALL_SENSORS) == 0)
            {
                nextState = APP_STATE_AUTONOMY;
            }
            break;

        case (APP_STATE_FAULT):
            if (supervisor_data.battery_voltage >= BATTERY_STATUS)
            {
                supervisor_data.fault_flag &= ~APP_FAULTS_LOW_BATTERY;
                nextState = APP_STATE_IDLE;
            }
            // TODO: maybe utilize the state when there are more faults??
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

static bool app_supervisor_private_transitToNewState(app_state_E state)
{
    switch (state)
    {
        case (APP_STATE_IDLE):
#if (FEATURE_PERIPHERALS)       
            dev_led_clear_leds();
#endif
#if (FEATURE_SUPER_CMD_DEV_DRIVER)    
            dev_avr_driver_set_req_Robot_motion(ROBOT_MOTION_BREAK, MOTOR_PWM_DUTY_40_PERCENT, MOTOR_PWM_DUTY_40_PERCENT);
#endif //(FEATURE_SUPER_CMD_DEV_DRIVER)    
            break;

        case (APP_STATE_AUTONOMY):
#if (FEATURE_PERIPHERALS)       
            // TODO: Debug why led not being set
            dev_led_clear_leds();
            dev_led_green_set(true);
#endif
#if (FEATURE_SUPER_CMD_DEV_DRIVER)    
            dev_avr_driver_set_req_Robot_motion(ROBOT_MOTION_FW_COAST, MOTOR_PWM_DUTY_40_PERCENT, MOTOR_PWM_DUTY_40_PERCENT);
#endif // (FEATURE_SUPER_CMD_DEV_DRIVER)    
#if (FEATURE_UV)
            dev_uv_fw_shutdown_clear();
            dev_uv_set_row(UV_PWM, UV_DAC);
#endif            
            break;

        case (APP_STATE_AUTONOMY_ESTOPPED):
#if (FEATURE_SUPER_USE_HARDCODE_CHORE)
            supervisor_data.estop_chorography_tick_20ms = APP_CHOREOGRAPHY_INIT;
            supervisor_data.estop_choreography_wip = APP_CHOREOGRAPHY_INIT;
#endif // (FEATURE_SUPER_USE_HARDCODE_CHORE)
#if (FEATURE_PERIPHERALS)               
            dev_led_clear_leds();
            dev_led_red_set(true);
#endif
#if (!FEATURE_SUPER_USE_HARDCODE_CHORE)
            dev_avr_driver_set_req_Robot_motion(ROBOT_MOTION_BREAK, MOTOR_PWM_DUTY_40_PERCENT, MOTOR_PWM_DUTY_40_PERCENT);
#endif
            break;

        case (APP_STATE_COUNT):            
        case (APP_STATE_UNKNOWN):
        default:
            supervisor_data.fault_flag |= APP_FAULTS_INVALID_STATE;
            break;
    }
    return true;
}

static bool app_supervisor_private_exitCurrentState(app_state_E state)
{
    switch (state)
    {
        case (APP_STATE_IDLE):
#if (FEATURE_SLAM)
            app_slam_requestToResetMap(); // reset map upon entering autonomy
#endif //(FEATURE_SLAM)
            break;

        case (APP_STATE_AUTONOMY_ESTOPPED):
        case (APP_STATE_AUTONOMY):
#if (FEATURE_UV)
            dev_uv_fw_shutdown();
#endif        
        case (APP_STATE_COUNT):            
        case (APP_STATE_UNKNOWN):
        default:
            // Do nothing
            break;
    }
    return true;
}

static bool app_supervisor_private_stateAction(app_state_E state)
{
#if (FEATURE_SUPER_USE_HARDCODE_CHORE)
    const uint8_t index = supervisor_data.estop_choreography_wip;
    const uint8_t mode  = supervisor_data.estop_choreography_sequence_mode[index];
    const uint8_t model = supervisor_data.estop_choreography_sequence_l   [index];
    const uint8_t moder = supervisor_data.estop_choreography_sequence_r   [index];
    const uint16_t tof  = supervisor_data.app_slam_EFlag;
    switch (state)
    {
        case (APP_STATE_AUTONOMY_ESTOPPED):
            if (index < APP_CHOREOGRAPHY_STEP_COUNT)
            {
#   if (DEBUG_FPRINT_APP_SUPERVISOR_STATE)
                PRINTF(">> output [%d] m:%d, l:%d, r:%d, tof:%d\n", index, mode, model, moder, tof);
#   endif // (DEBUG_FPRINT_APP_SUPERVISOR)
                // perform action sequence:
#   if (FEATURE_SUPER_CMD_DEV_DRIVER)    
                dev_avr_driver_set_req_Robot_motion(mode, model, moder);
#   endif //(FEATURE_SUPER_CMD_DEV_DRIVER)    
                supervisor_data.estop_chorography_tick_20ms ++;
                supervisor_data.estop_choreography_wip = supervisor_data.estop_chorography_tick_20ms/CHOREOGRAPHY_BASE_TICK_20MS;
            }
            break;

        case (APP_STATE_IDLE):
        case (APP_STATE_AUTONOMY):
        case (APP_STATE_COUNT):            
        case (APP_STATE_UNKNOWN):
        default:
            // Do nothing
            break;
    }
#endif // (FEATURE_SUPER_USE_HARDCODE_CHORE)
    return true;
}

static void app_supervisor_private_fetchState(void)
{
#if (FEATURE_PERIPHERALS)
    supervisor_data.button_pressed = dev_button_update_50ms();
#endif
#if (FEATURE_SENSOR_AVR)
    supervisor_data.avr_sensor_data = dev_avr_sensor_uart_get();
    // PRINTF("AVR SENSOR DATE: %c%c%c%c%c%c%c%c\n", BYTE_TO_BINARY(supervisor_data.avr_sensor_data));
#endif
#if (FEATURE_SUPER_USE_PROFILED_MOTIONS | MOCK)
    // uint8_t frame=0U;
    // int8_t rmotor, lmotor;
    // frame = app_slam_getMotionVelocity(& lmotor, & rmotor, frame);
    // PRINTF("[MOTOR] R:%3d, L:%3d (f:%3d) \n", lmotor, rmotor, frame);
#endif
    // TODO: battery status
#if (FEATURE_BATTERY)
    supervisor_data.battery_voltage = dev_battery_get();
#endif    
    // TODO: IMU

    // APP_SLAM
    supervisor_data.app_slam_EFlag = app_slam_requestToFDangerZone();
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_supervisor_init(void)
{
    // do nothing
}

// TODO: Rename to 20ms
void app_supervisor_run50ms(void)
{
    app_state_E current_state = supervisor_data.current_state;

    app_supervisor_private_fetchState();

    app_state_E next_state = app_supervisor_private_getNextState(current_state);

    if (next_state != current_state)
    {
        app_supervisor_private_exitCurrentState(current_state);
#if (DEBUG_FPRINT_APP_SUPERVISOR)
        PRINTF("STATE TRANSITION from %d to %d\n", current_state, next_state);
#endif //(DEBUG_FPRINT_APP_SUPERVISOR)
        app_supervisor_private_transitToNewState(next_state);
        supervisor_data.current_state = next_state;
    }
    app_supervisor_private_stateAction(current_state);

}

