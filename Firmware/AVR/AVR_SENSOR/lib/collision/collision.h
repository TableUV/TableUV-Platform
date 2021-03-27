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

typedef struct{
#if (COL_INT_ENABLED)    
    volatile collision_status_t col_status;
#endif //COL_INT_ENABLED    
    bool left_col_pressed;
    bool right_col_pressed;
} collision_data_S;

void collision_init(void);
uint8_t collision_status_get(void);
collision_data_S collision_status_retrieve(void);
bool col_pressed_get(void);
void col_pressed_clear(void);
void collision_test(void);


#endif //DEV_BATTERY_H