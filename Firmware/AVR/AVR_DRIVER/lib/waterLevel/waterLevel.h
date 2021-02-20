
/**
 * @file waterLevel.h
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for waterLevel sensor 
 *
 */

#ifndef _WATERLEVEL_H_
#define _WATERLEVEL_H_


#ifdef __cplusplus
extern "C"{
#endif 

#include <stdio.h>
#include "../../include/pinConfig.h"


void setupWaterLevelConfig();
uint8_t getWaterLevelSignal(); 


#ifdef __cplusplus  
}
#endif 

#endif


