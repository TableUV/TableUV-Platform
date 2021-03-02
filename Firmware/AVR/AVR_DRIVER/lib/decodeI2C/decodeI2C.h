
/**
 * @file motor.h
 * @author Tsugumi Murata (tmurata293)
 * @date 20 Feb 2021
 * @brief library for decoding I2C message  
 *
 */

#ifndef _DECODEI2C_H_
#define _DECODEI2C_H_


#ifdef __cplusplus
extern "C"{
#endif 

#include <stdio.h>
#include "../../include/pinConfig.h"
#include "../../include/avr_driver_common.h"

uint8_t checkDataHeader(char byteData, data_frame_header_E data_frame_header); 

#ifdef __cplusplus  
}
#endif 

#endif
