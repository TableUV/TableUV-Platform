/**
 * @file common.h
 * @author Tsugmui Murata
 * @date 1 Mar 2021
 * @brief AVR DRIVER COMMON HEADER FILE
 * @version V1.1
 *
 * This document will contains ping definitions
 */

#ifndef AVR_DRIVER_COMMON_H
#define AVR_DRIVER_COMMON_H
#ifdef __cplusplus
extern "C"{
#endif 

/////////////////////////////////
/////////   INCLUDE     /////////
/////////////////////////////////
#include <stdint.h>


/////////////////////////////////
/////////   MACRO     ///////////
/////////////////////////////////

// i2c address for both avr driver 
#define LEFT_AVR_DRIVER_I2C_ADDRESS     (0x0A)
#define RIGHT_AVR_DRIVER_I2C_ADDRESS    (0x2A)

//16 bit data mask 
#define DATA_MASK_16BIT_FIRST_8BIT      (0xFF00)
#define DATA_MASK_16BIT_SECOND_8BIT     (0x00FF)

//32 bit data mask
#define DATA_MASK_32BIT_FIRST_8BIT      (0xFF000000)
#define DATA_MASK_32BIT_SECOND_8BIT     (0x00FF0000)
#define DATA_MASK_32BIT_THIRD_8BIT      (0x0000FF00)
#define DATA_MASK_32BIT_FOURTH_8BIT     (0x000000FF)

// check for data validity 
#define DATA_VALIDITY_MASK              (_BV(7) | _BV(6))

// data mask for first incoming data byte 
#define ESTOP_COMMAND_REQ_MASK          (_BV(5))

#define HAPTIC_EN_REQ_MASK              (_BV(3))

#define TOF_XSHUT_EN_REQ_BIT_MASK       (_BV(1) | _BV(0))

// data mask for second incoming data byte 
#define MOTOR_MODE_REQ_MASK             (_BV(5))
#define MOTOR_DIRECTION_REQ_MASK        (_BV(4))
#define MOTOR_PWM_DUTY_REQ_MASK         (_BV(3) | _BV(2) | _BV(1) | _BV(0))


// data frame header
typedef enum data_frame_header{
    DATA_FRAME_HEADER_FIRST,
    DATA_FRAME_HEADER_SECOND,
    DATA_FRAME_HEADER_THIRD,
    DATA_FRAME_HEADER_FOURTH,
    DATA_FRAME_HEADER_COUNT,
    DATA_FRAME_HEADER_UNDEFINED
}data_frame_header_E;

// motor modes
typedef enum motor_mode{
    MOTOR_MODE_COAST,
    MOTOR_MODE_CW_COAST,
    MOTOR_MODE_CCW_COAST,
    MOTOR_MODE_CW_BREAK,
    MOTOR_MODE_CCW_BREAK,
    MOTOR_MODE_BREAK,
    MOTOR_MODE_HEADER_COUNT,
    MOTOR_MODE_HEADER_UNDEFINED
} motor_mode_E; 

// motor_pwm_duty
typedef enum motor_pwm_duty{
    MOTOR_PWM_DUTY_0_PERCENT, 
    MOTOR_PWM_DUTY_10_PERCENT,
    MOTOR_PWM_DUTY_20_PERCENT,
    MOTOR_PWM_DUTY_30_PERCENT,
    MOTOR_PWM_DUTY_40_PERCENT,
    MOTOR_PWM_DUTY_50_PERCENT,
    MOTOR_PWM_DUTY_60_PERCENT,
    MOTOR_PWM_DUTY_70_PERCENT,
    MOTOR_PWM_DUTY_80_PERCENT,
    MOTOR_PWM_DUTY_90_PERCENT,
    MOTOR_PWM_DUTY_100_PERCENT,
    MOTOR_PWM_DUTY_COUNT,
    MOTOR_PWM_DUTY_UNDEFINED
} motor_pwm_duty_E;

// motor_command_mode 
typedef enum motor_command_mode{
    MOTOR_COMMAND_MODE_COAST,
    MOTOR_COMMAND_MODE_BRAKE,
    MOTOR_COMMAND_COUNT,
    MOTOR_COMMAND_UNDEFINED
} motor_command_mode_E; 

// motor_command_direction 
typedef enum motor_command_direction{
    MOTOR_COMMAND_DIRECTION_CCW,
    MOTOR_COMMAND_DIRECTION_CW,
    MOTOR_COMMAND_DIRECTION_COUNT,
    MOTOR_COMMAND_DIRECTION_UNDEFINED
} motor_command_direction_E; 

// tof_sensor_config
// XSHUT is active low, pulling low shutsdown the sensor 
typedef enum tof_sensor_config{
    TOF_SENSOR_CONFIG_DISABLE_ALL,  //pull XSHUT low, sensor shutdown (active low )
    TOF_SENSOR_CONFIG_ENABLE_1,
    TOF_SENSOR_CONFIG_ENABLE_2,
    TOF_SENSOR_CONFIG_ENABLE_3,
    TOF_SENSOR_CONFIG_COUNT,
    TOF_SENSOR_CONFIG_UNDEFINED
} tof_sensor_config_E; 


#ifdef __cplusplus  
}
#endif 
#endif //AVR_DRIVER_COMMON_H

