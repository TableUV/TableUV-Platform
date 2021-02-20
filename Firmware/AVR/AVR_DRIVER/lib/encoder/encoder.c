
/**
 * @file encoder.c
 * @author Tsugumi Murata (tmurata293)
 * @date 18 Feb 2021
 * @brief library for encoder 
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>  
#include "encoder.h"

volatile int32_t encod_count           = 0x00;
volatile encoder_state_t encod_curr    = ZERO_ZERO;
volatile encoder_state_t encod_prev    = ZERO_ZERO;


static encoder_state_t getCurrentPhase(){
    return ((PINA) & (ENCODER_SIG_MASK)); 
}

/*********************************
             ISR 
**********************************/
ISR(PCINT0_vect){
    cli(); 
    encod_curr = getCurrentPhase(); 
    switch(encod_prev){
        case(ZERO_ZERO):
            if (encod_curr == ONE_ZERO) encod_count ++; 
            else if(encod_curr == ZERO_ONE) encod_count --; 
        break;
        case(ZERO_ONE):
            if (encod_curr == ZERO_ZERO) encod_count ++; 
            else if(encod_curr == ONE_ONE) encod_count --; 
        break;
        case(ONE_ZERO):
            if (encod_curr == ONE_ONE) encod_count ++; 
            else if(encod_curr == ZERO_ZERO) encod_count --; 
        break;
        case(ONE_ONE):
            if (encod_curr == ZERO_ONE) encod_count ++; 
            else if(encod_curr == ONE_ZERO) encod_count --; 
        break;
    }
    encod_prev = encod_curr;
    sei();
}

// function
void setupEncoderConfig(){

    // pin setup for left motor 
    DDRA &= ~_BV(ENCODER_SIG_A ) & ~_BV(ENCODER_SIG_B); 

    // interupt setting 
    GIMSK  |= _BV(PCIE0);  
    SREG   |= _BV(7); 
    PCMSK0 |= _BV(ENCODER_SIG_A) | _BV(ENCODER_SIG_B );

    // get current encoder phase 
    encod_prev  = getCurrentPhase(); 

    // set current count to 0 
    setEncoderCount(0);
}

void setEncoderCount(uint8_t count){
    encod_count = count; 
}

int8_t getEncoderCount8(){
    return encod_count;
}

int8_t getEncoderCount16_first_8bit(){
    return ((encod_count & ENCODER16_FIRST_8BIT) >> 8);
}
int8_t getEncoderCount16_second_8bit(){
    return (encod_count & ENCODER16_SECOND_8BIT);
}

int8_t getEncoderCount32_first_8bit(){
    return ((encod_count & ENCODER32_FIRST_8BIT) >> 24);
}

int8_t getEncoderCount32_second_8bit(){
    return ((encod_count & ENCODER32_SECOND_8BIT) >> 16);
}

int8_t getEncoderCount32_third_8bit(){
    return ((encod_count & ENCODER32_THIRD_8BIT) >> 8);
}

int8_t getEncoderCount32_fourth_8bit(){
    return (encod_count & ENCODER32_FOURTH_8BIT);
}
