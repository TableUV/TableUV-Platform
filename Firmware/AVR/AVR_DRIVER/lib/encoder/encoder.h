
/**
 * @file encoder.h
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for encoder 
 *
 */

#ifndef _ENCODER_H_
#define _ENCODER_H_


#ifdef __cplusplus
extern "C"{
#endif 

#include <stdio.h>
#include "../../include/pinConfig.h"

#define ENCODER_SIG_MASK (_BV(ENCODER_SIG_A) | _BV(ENCODER_SIG_B) )

#define ENCODER16_FIRST_8BIT             0xFF00
#define ENCODER16_SECOND_8BIT            0x00FF

#define ENCODER32_FIRST_8BIT             0xFF000000
#define ENCODER32_SECOND_8BIT            0x00FF0000
#define ENCODER32_THIRD_8BIT             0x0000FF00
#define ENCODER32_FOURTH_8BIT            0x000000FF

typedef enum {
    ZERO_ZERO,
    ZERO_ONE ,
    ONE_ZERO ,
    ONE_ONE
} encoder_state_t;

// public function
void setupEncoderConfig();
void setEncoderCount(uint8_t count); 
int8_t getEncoderCount8(); 
int8_t getEncoderCount16_first_8bit(); 
int8_t getEncoderCount16_second_8bit(); 
int8_t getEncoderCount32_first_8bit(); 
int8_t getEncoderCount32_second_8bit(); 
int8_t getEncoderCount32_third_8bit(); 
int8_t getEncoderCount32_fourth_8bit(); 

#ifdef __cplusplus  
}
#endif 

#endif