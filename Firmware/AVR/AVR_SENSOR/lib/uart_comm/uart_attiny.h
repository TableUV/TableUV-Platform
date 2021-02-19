/**
 * @file uart_attiny.h
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */
#ifndef UART_ATTINY_H
#define UART_ATTINY_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

#include "../../include/pinConfig.h"


void uart_attiny_init(void);
void UART_tx(char character);
void UART_tx_str(char* string);
void uart_test_code(void);


#endif //UART_ATTINY_H