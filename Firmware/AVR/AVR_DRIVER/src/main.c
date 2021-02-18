/**
 * @file main.c
 * @author Tsugumi Murata (tmurata293)
 * @date 17 Feb 2021
 * @brief Main File for AVR DRIVER
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>    
#include <util/delay.h>      
#include <stdint.h>                 
#include "usiTwiSlave.h"            
#include "pinConfig.h"
#include "motor.h"
#include "encoder.h"


/*********************************
    select which driver avr  
**********************************/
//comment out this line if coding for right avr 

//#define USE_LEFT_DRIVER_AVR TRUE


#ifdef USE_LEFT_DRIVER_AVR
    #define DRIVER_AVR      0
    #define SLAVE_ADDRESS   0x55    //slave address for left driver avr 
#else
    #define DRIVER_AVR      1
    #define SLAVE_ADDRESS   0x56    //slave address for right driver avr 
#endif 


/*********************************
             ISR 
**********************************/
ISR(PCINT0_vect){

}


int main(void)
{
    // pin setup 
    DDRA |= ( PIN_DIR_INPUT << ENCODER_SIG_A ) | ( PIN_DIR_INPUT << ENCODER_SIG_B ) | ( PIN_DIR_OUTPUT << TWI_I2C_SCL ) | ( PIN_DIR_OUTPUT << TWI_I2C_SDA) | ( PIN_DIR_OUTPUT << MOTOR_OUT_B); 
    DDRB |= ( PIN_DIR_INPUT << MODE_SELECT) | ( PIN_DIR_OUTPUT << MOTOR_OUT_A); 

    //setup encoder interrupt settings 
    setupInterruptEncoder(); 

    //Set up the USI communicatin
    usiTwiSlaveInit(SLAVE_ADDRESS);

       while(1)
    { 
        //if data received from master
        if(usiTwiDataInReceiveBuffer()){
            
            switch(usiTwiReceiveByte()){
                //trigger signal detected from master
                case 'r':
                cli();                              //disable interrupt
                
                //send encod_count to master (first 8 bit)
                usiTwiTransmitByte( (encod_count & FIRST_8BIT) >> 8 );
                //send encod_count to master (second 8 bit)
                usiTwiTransmitByte( (encod_count & SECOND_8BIT ));
                encod_count = 0;                    //reset the count
                
                sei();                              //enable interrupt
                break;
            }
        }
        
        
    }
    return 0;   /* never reached */
}

