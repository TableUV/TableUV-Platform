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
#include "waterLevel.h"
#include "mistActuator.h"

int main(void)
{
    
    // check which driver avr it is 
    DDRB &= ~_BV(MODE_SELECT); 
    uint8_t driver_mode = (PINB & _BV(MODE_SELECT));

    uint8_t driver_avr = 0;
    uint8_t slave_address = 0x2A;  

    DDRB |= _BV(PB0);
    PORTB ^= _BV(PB0);
    //left driver 
    if (driver_mode) {
        driver_avr = 1; 
        slave_address = 0x2A;   //slave address for left driver avr 
        // pin setup 
        DDRA |= _BV(TWI_I2C_SCL ) | _BV(TWI_I2C_SDA); 
    }
    //right driver 
    else{
        driver_avr = 0;
        slave_address = 0x0A ;   //slave address for right driver avr 
        // pin setup 
        DDRA |= _BV(TWI_I2C_SCL ) | _BV(TWI_I2C_SDA); 
    }

    //setup encoder interrupt settings 
    setupEncoderConfig(); 
    setupMotorConfig(); 

    //setup only needed for right avr driver 
    if(!driver_avr){
        setupWaterLevelConfig();
        setupMistActuatorConfig();
    }

    //Set up the USI communicatin
    usiTwiSlaveInit(slave_address);

       while(1)
    { 
        // _delay_ms(500);

        //if data received from master
        if(usiTwiDataInReceiveBuffer()){

            switch(usiTwiReceiveByte()){
                //trigger signal detected from master
                case 'r':
                cli();
                //send encod_count to master (first 8 bit)
                usiTwiTransmitByte(getEncoderCount16_first_8bit());
                //send encod_count to master (second 8 bit)
                usiTwiTransmitByte(getEncoderCount16_second_8bit());
                //setEncoderCount(0);                   //reset the count
                
                sei();                              //enable interrupt
                break;
            }
        }
        
        
    }
    return 0;   /* never reached */
}

