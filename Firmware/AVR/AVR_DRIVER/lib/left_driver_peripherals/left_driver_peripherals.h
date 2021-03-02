/**
 * @file left_driver_peripherals.h
 * @author Tsugumi Murata (tmurata293)
 * @date 20 Feb 2021
 * @brief library for left avr driver peripherals 
 */

#ifndef _LEFT_DRIVER_PERIPHERALS_H_
#define _LEFT_DRIVER_PERIPHERALS_H_


#ifdef __cplusplus
extern "C"{
#endif 

#include <stdio.h>
#include "../../include/pinConfig.h"

void setup_TOF_XSHUT_Config();
void disable_TOF_XSHUT_1();
void disable_TOF_XSHUT_2();
void disable_TOF_XSHUT_3();
void enable_TOF_XSHUT_All();

#ifdef __cplusplus  
}
#endif 

#endif