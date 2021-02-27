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
    CLKPR = _BV(CLKPCE); 
    CLKPR = 0;

    collision_init();
    uart_attiny_init();

    // Test code
    char test_array[8] = {'1', '2', '3', '4', '5', '6', '7', '8'};

    while(1)
    {
        // collision_status_retrieve();
        // ir_sensor_retrieve();
        UART_tx_str(test_array, 8);
        _delay_ms(100);
    }
}