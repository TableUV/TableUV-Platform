/**
 * @file motor.c
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for motor 
 *
 */

#include <avr/io.h>
#include "motor.h"

void setupMotorConfig(pwm_mode_t pwm_mode){
    DDRA |= _BV(MOTOR_OUT_B);
    DDRB |= _BV(MOTOR_OUT_A); 

    // set COM (compare output mode)
    TCCR0A |= _BV(COM0A1) | _BV(COM0B1); 
        //for fast pwm: 
            // Clear (output low) on Compare Match 
            // Set (output high) at BOTTOM (non-inverting mode)
        //for phase correct
            // Clear (output low) on Compare Match when up-counting.
            // Set (output high) Compare Match when down-counting.

    if (pwm_mode == FAST_PWM_MODE){
        // set WGM (waveform generation mode)
        TCCR0A |= _BV(WGM01) | _BV(WGM00);
        // set prescaler of 8, freq = 3.91khz
        TCCR0B |=  _BV(CS01);

    }
    else if (pwm_mode == PHASE_CORRECT_PWM){
        // set WGM (waveform generation mode)
        TCCR0A |= _BV(WGM00);
        // set prescaler of 8 , freq = 1.95khz (phase correct pwm is half freq of fast pwm mode)
        TCCR0B |= _BV(CS01);

    }
}

void setMotor(motor_mode_t motor_mode, motor_pwm_duty_t percent_pwm){
    switch(motor_mode){
        case(COAST):
            //PORTB &= ~_BV(MOTOR_OUT_A); 
            //PORTA &= ~_BV(MOTOR_OUT_B);
            OCR0A = 0; 
            OCR0B = 0; 
        break;

        case(FW_COAST):
            OCR0A = percent_pwm * REG_MAX / 10 ;
            OCR0B = 0; 
            //PORTA &= ~_BV(MOTOR_OUT_B);
        break; 

        case(REV_COAST):
            //PORTB &= ~_BV(MOTOR_OUT_A); 
            OCR0A = 0; 
            OCR0B = percent_pwm * REG_MAX / 10;
        break; 

        case(FW_BREAK):
            OCR0A = 255 ;
            OCR0B = percent_pwm * REG_MAX / 10;
        break; 

        case(REV_BREAK):
            OCR0A = percent_pwm * REG_MAX / 10 ;
            OCR0B = 255 ;
        break; 

        case(BREAK):
            OCR0A = 255;
            OCR0B = 255;
        break; 
    }
}

void eStopMotor(){
    OCR0A = 255;
    OCR0B = 255;
}

void testMotorAll(){
    setMotor(COAST, MOTOR_DUTY_50_PERCENT);
    _delay_ms(500);
    setMotor(FW_COAST, MOTOR_DUTY_50_PERCENT);
    _delay_ms(5000);
    setMotor(REV_COAST, MOTOR_DUTY_50_PERCENT);
    _delay_ms(500);
    setMotor(FW_BREAK, MOTOR_DUTY_50_PERCENT);
    _delay_ms(500);
    setMotor(REV_BREAK, MOTOR_DUTY_50_PERCENT);
    _delay_ms(500);
    setMotor(BREAK, MOTOR_DUTY_50_PERCENT);
    _delay_ms(500);
}
