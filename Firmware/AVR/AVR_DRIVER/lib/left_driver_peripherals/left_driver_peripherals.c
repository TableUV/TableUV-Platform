/**
 * @file left_driver_peripherals.c
 * @author Tsugumi Murata (tmurata293)
 * @date 20 Feb 2021
 * @brief library for left avr driver peripherals 
 *      includes - TOF config pin
 */

#include <avr/io.h>
#include <avr/interrupt.h>  
#include "left_driver_peripherals.h"


void setup_TOF_XSHUT_Config(){
    DDRA |= _BV(TOF_CONFIG_1) | _BV(TOF_CONFIG_2) | _BV(TOF_CONFIG_3); 
}

void disable_TOF_XSHUT_1(){
    PORTA |= _BV(TOF_CONFIG_1);
}
void disable_TOF_XSHUT_2(){
    PORTA |= _BV(TOF_CONFIG_2);
}
void disable_TOF_XSHUT_3(){
    PORTA |= _BV(TOF_CONFIG_3);
}
void enable_TOF_XSHUT_All(){
    PORTA &= ~_BV(TOF_CONFIG_1) & ~_BV(TOF_CONFIG_2) & ~_BV(TOF_CONFIG_3);

}
