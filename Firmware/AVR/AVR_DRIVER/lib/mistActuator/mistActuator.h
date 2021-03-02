
/**
 * @file mistActuator.h
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for mistActuator sensor 
 *
 */

#ifndef _MISTACTUATOR_H_
#define _MISTACTUATOR_H_


#ifdef __cplusplus
extern "C"{
#endif 

#include <stdio.h>
#include "../../include/pinConfig.h"

void setupMistActuatorConfig();
void enableMistActuator(); 
void disableMistActuator(); 

#ifdef __cplusplus  
}
#endif 

#endif
