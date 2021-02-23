/**
 * @file io_ping_map.h
 * @author Tsugmui Murata
 * @date 15 Feb 2021
 * @brief IO Ping Map Configuration
 *
 * This document will contains ping definitions
 */

#ifndef IO_GPIO_MAP_H
#define IO_GPIO_MAP_H


#ifdef __cplusplus
extern "C"{
#endif 


// GPIO 
#define SWITCH               36
#define TOF_SHUT             25
#define STATUS_GREEN_LED     12
#define STATUS_RED_LED       13
#define CHARGE_STATUS        GPIO_NUM_15
#define BATTERY_VOLTAGE      GPIO_NUM_2
#define FW_SHUTDOWN          18

// INT
#define TOF_INT_3            39
#define TOF_INT_2            35
#define TOF_INT_1            34

// DAC 
#define ESP_DAC              DAC_CHANNEL_2  //DAC_CHANNEL_2

//ADC 

//I2C
#define TOF_I2C_SCL          32
#define TOF_I2C_SDA          33
#define MOTOR_I2C_SCL        22
#define MOTOR_I2C_SDA        23

//UART 
#define TOF_UART_ESP_RX      27
#define TOF_UART_ESP_TX      14

//PWM 
#define LED_ROW_MODULATE     19
#define LED_SIDE_MODULATE    21

//SPI 
#define CS_3V3               4
#define SDO_3V3              16
#define SCLK_3V3             17
#define SDI_3V3              5


#ifdef __cplusplus  
}
#endif 
#endif //IO_GPIO_MAP_H