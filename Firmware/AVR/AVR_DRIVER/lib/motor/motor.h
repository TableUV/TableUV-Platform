
/**
 * @file motor.h
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for motor 
 *
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_


#ifdef __cplusplus
extern "C"{
#endif 

#include <stdio.h>
#include "../../include/pinConfig.h"

#define REG_MAX 255

typedef enum motor_mode{
    COAST,
    FW_COAST,
    REV_COAST,
    FW_BREAK,
    REV_BREAK,
    BREAK
} motor_mode_t; 

typedef enum motor_pwm_duty{
    MOTOR_DUTY_0_PERCENT, 
    MOTOR_DUTY_10_PERCENT,
    MOTOR_DUTY_20_PERCENT,
    MOTOR_DUTY_30_PERCENT,
    MOTOR_DUTY_40_PERCENT,
    MOTOR_DUTY_50_PERCENT,
    MOTOR_DUTY_60_PERCENT,
    MOTOR_DUTY_70_PERCENT,
    MOTOR_DUTY_80_PERCENT,
    MOTOR_DUTY_90_PERCENT,
    MOTOR_DUTY_100_PERCENT
} motor_pwm_duty_t;


typedef enum pwm_mode{
    FAST_PWM_MODE,
    PHASE_CORRECT_PWM
} pwm_mode_t;

void setupMotorConfig(pwm_mode_t pwm_mode); 
void setMotor(motor_mode_t motor_mode, motor_pwm_duty_t percent_pwm); 
void eStopMotor(); 
void testMotorAll(); 


#ifdef __cplusplus  
}
#endif 

#endif





