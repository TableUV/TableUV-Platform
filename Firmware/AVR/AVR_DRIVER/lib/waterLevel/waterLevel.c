/**
 * @file waterLevel.c
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for waterLevel 
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>  
#include "waterLevel.h"



void setupWaterLevelConfig(){
    DDRA &= ~_BV(WATER_LEVEL_SIG); 
}

uint8_t getWaterLevelSignal(){
    //tbd
    return 1;
}