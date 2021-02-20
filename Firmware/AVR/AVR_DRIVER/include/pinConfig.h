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


#define MOTOR_OUT_A         PB2     //output, OC0A
#define MOTOR_OUT_B         PA7     //output, OC0B

#define ENCODER_SIG_A       PA0     //input, PCINT0
#define ENCODER_SIG_B       PA1     //input, PCINT1

#define MODE_SELECT         PB1     //input

#define TWI_I2C_SDA         PA6     //bi-directional, SDA
#define TWI_I2C_SCL         PA4     //output, SCL 

#define STATUS_LED          PA2     //output
#define MIST_ACTUATOR       PA3     //output
#define WATER_LEVEL_SIG     PA5     //input, ADC

#ifdef __cplusplus  
}
#endif 


#endif 