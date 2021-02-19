/**
 * @file collision.h
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */
#include <avr/io.h>
#include <avr/interrupt.h>  
#include <stdbool.h>
#include <stdint.h>


#ifndef COLLISION_H
#define COLLISION_H
#include "../../include/pinConfig.h"

void collision_init(void);
uint8_t collision_status_get(void);
uint8_t collision_status_retrieve(void);
bool col_pressed_get(void);
bool col_pressed_clear(void);
void collision_test(void);


#endif //DEV_BATTERY_H