
/**
 * @file encoder.c
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for encoder 
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>  
#include "encoder.h"
#include "pinConfig.h"



// function
void setupInterruptEncoder(){
    GIMSK  |= ( 1 << PCIE0);  
    SREG   |= ( 1 << 7 ); 
    PCMSK0 |= ( 1 << ENCODER_SIG_A ) | ( 1 << ENCODER_SIG_B );
}
