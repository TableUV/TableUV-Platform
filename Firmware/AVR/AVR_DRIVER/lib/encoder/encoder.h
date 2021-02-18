
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

typedef enum {
    ZERO_ZERO,
    ZERO_ONE,
    ONE_ZERO,
    ONE_ONE
} encoder_state_t;


// function
void setupInterruptEncoder();


#ifdef __cplusplus  
}
#endif 

#endif