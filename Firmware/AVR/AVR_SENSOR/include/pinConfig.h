/**
 * @file pinConfig.h
 * @author Tsugumi Murata (tmurata293)
 * @date 17 Feb 2021
 * @brief Pin Config File for AVR DRIVER
 *
 */


#ifndef _PINCONFIG_H_
#define _PINCONFIG_H_


#ifdef __cplusplus
extern "C"{
#endif 


#define PIN_DIR_INPUT       0
#define PIN_DIR_OUTPUT      1

#define RIGHT_COLLISION     PB0     // PCINT8
#define LEFT_COLLISION      PB1     // PCINT9

#define IR_FRONT_1          PA7
#define IR_FRONT_2          PA6
#define IR_RIGHT_1          PA4
#define IR_RIGHT_2          PA5
#define IR_LEFT_1           PA0
#define IR_LEFT_2           PA3

#define UART_TX             PA2
#define TX_PORT             PORTA
#define TX_DDR              DDRA
#define TX_DDR_PIN          DDA2                      

#define UART_RX             PA1   

#ifdef __cplusplus  
}
#endif 


#endif 