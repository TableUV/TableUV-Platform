
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
#include "../../include/avr_driver_common.h"

#define REG_MAX 255

typedef enum pwm_mode{
    PWM_MODE_FAST,
    PWM_MODE_PHASE_CORRECT
} pwm_mode_E;

void setupMotorConfig(pwm_mode_E pwm_mode); 
void setMotor(motor_mode_E motor_mode, motor_pwm_duty_E percent_pwm); 
void eStopMotor(); 
void testMotorAll(); 


#ifdef __cplusplus  
}
#endif 

#endif





