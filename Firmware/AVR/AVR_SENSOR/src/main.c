/**
 * @file main.c
 * @author 
 * @date 18 Feb 2021
 * @brief Main File for AVR SENSOR
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>    
#include <util/delay.h>      
#include <stdint.h>  

#include "../include/pinConfig.h"
#include "collision.h"
#include "uart_attiny.h"
#include "ir_sensor.h"

int main()
{
    collision_init();
    uart_attiny_init();
}