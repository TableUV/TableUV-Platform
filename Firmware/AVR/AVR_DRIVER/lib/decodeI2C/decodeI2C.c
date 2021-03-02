/**
 * @file motor.c
 * @author Tsugumi Murata (tmurata293)
 * @date 20 Feb 2021
 * @brief library for decoding i2c message 
 *
 */

#include <avr/io.h>
#include "decodeI2C.h"


uint8_t checkDataHeader(char byteData, data_frame_header_E frameNumber){
    return ( ((byteData & DATA_VALIDITY_MASK)  >> 6) == frameNumber);
}
