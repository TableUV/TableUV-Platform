/**
 * @file motor.c
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for motor 
 *
 */

#include <avr/io.h>
#include "motor.h"

void setupMotorConfig(){
    DDRA |= _BV(MOTOR_OUT_B);
    DDRB |= _BV(MOTOR_OUT_A); 
}

void setMotor(motor_modo_t motor_mode, uint8_t percent_pwm){
    // OCR0A |= percent_pwm / 100 * REG_MAX ;
    // OCR0B |= percent_pwm / 100 * REG_MAX ;
    // switch(motor_mode){
    //     case(COAST):
    //         TCCR0A |=  _BV(COM0A0);
    //         TCCR0A |=  _BV(COM0A1);

    // }

}

void eStopMotor(){

}
