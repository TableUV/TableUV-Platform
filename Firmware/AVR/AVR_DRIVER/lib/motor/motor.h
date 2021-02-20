
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
    REB_BREAK,
    BREAK
} motor_modo_t; 


void setupMotorConfig(); 
void setMotor(motor_modo_t motor_mode, uint8_t percent_pwm); 
void eStopMotor(); 

#ifdef __cplusplus  
}
#endif 

#endif





