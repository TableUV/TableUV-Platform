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


void ir_attiny_init(void);
void ir_test_code(void);


#endif // IR_SENSOR_H