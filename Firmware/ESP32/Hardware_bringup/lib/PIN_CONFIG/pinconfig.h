/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : PINCONFIG.H
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: pinconfig file for ESP32 
#####################################################################
*/

#ifndef _PINCONFIG_H_
#define _PINCONFIG_H_


#ifdef __cplusplus
extern "C"{
#endif 


// GPIO 
#define SWITCH               36
#define TOF_SHUT             25
#define STATUS_GREEN_LED     12
#define STATUS_RED_LED       13
#define CHARGE_STATUS        15
#define BATTERY_VOLTAGE      2
#define FW_SHUTDOWN          18

// INT
#define TOF_INT_3            39
#define TOF_INT_2            34
#define TOF_INT_1            35

// DAC 
#define ESP_DAC              26

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


// function
void pin_setup();


#ifdef __cplusplus  
}
#endif 


#endif 