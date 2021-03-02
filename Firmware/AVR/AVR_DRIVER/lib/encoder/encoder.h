
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


typedef enum {
    ENCODER_PHASE_ZERO_ZERO,
    ENCODER_PHASE_ZERO_ONE ,
    ENCODER_PHASE_ONE_ZERO ,
    ENCODER_PHASE_ONE_ONE
} encoder_state_E;

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