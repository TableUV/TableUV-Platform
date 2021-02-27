/**
 * @file ir_sensor.h
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include <stdio.h>

#include "../../include/pinConfig.h"


typedef struct{
    volatile uint8_t ir_front_1_data;
    volatile uint8_t ir_front_2_data;
    volatile uint8_t ir_right_1_data;
    volatile uint8_t ir_right_2_data;
    volatile uint8_t ir_left_1_data;
    volatile uint8_t ir_left_2_data;
} ir_data_S;


void ir_attiny_init(void);
void ir_sensor_update(void);
ir_data_S ir_sensor_get(void);
ir_data_S ir_sensor_retrieve(void);
void ir_test_code(void);


#endif // IR_SENSOR_H