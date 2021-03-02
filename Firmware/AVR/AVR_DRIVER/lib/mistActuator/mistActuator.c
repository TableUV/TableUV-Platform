/**
 * @file mistActuator.c
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for mistActuator sensor 
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>  
#include "mistActuator.h"


void setupMistActuatorConfig(){
    DDRA |= _BV(MIST_ACTUATOR ) | _BV(STATUS_LED); 
    disableMistActuator();
}

void enableMistActuator(){
    PORTA |= _BV(MIST_ACTUATOR) | _BV(STATUS_LED);
} 

void disableMistActuator(){
    PORTA &= ~_BV(MIST_ACTUATOR) & ~_BV(STATUS_LED);
}