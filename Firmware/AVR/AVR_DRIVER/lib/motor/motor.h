
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

typedef enum motor_mode{
    COAST,
    FW_COAST,
    REV_COAST,
    FW_BREAK,
    REB_BREAK,
    BREAK
} motor_modo_t; 



#ifdef __cplusplus  
}
#endif 

#endif





