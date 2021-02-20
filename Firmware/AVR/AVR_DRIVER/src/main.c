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


static uint8_t getDriverMode(){
    // check which driver avr it is 
    DDRB &= ~_BV(MODE_SELECT); 
    return (PINB & _BV(MODE_SELECT));
}


int main(void)
{
    // set system clock to 8MHz
    CLKPR  = _BV(CLKPCE);
    CLKPR  = 0;
    
    // define local variables 
    uint8_t driver_mode = 0; 
    uint8_t driver_avr = 0;
    uint8_t slave_address = 0x2A;  

    //check mode pin 
    driver_mode = getDriverMode(); 

    //left driver 
    if (driver_mode) {
        driver_avr = 1; 
        slave_address = 0x2A;   //slave address for left driver avr 
    }
    //right driver 
    else{
        driver_avr = 0;
        slave_address = 0x0A ;   //slave address for right driver avr 
        //setup only needed for right avr driver 
        setupWaterLevelConfig();
        setupMistActuatorConfig();
    }

    //setup usi twi settings 
    setupUsiTwiConfig();

    //setup encoder interrupt settings 
    setupEncoderConfig(); 
    //setup motor config 
    setupMotorConfig(PHASE_CORRECT_PWM); 

    //Set up the USI communicatin
    usiTwiSlaveInit(slave_address);

    DDRB |= _BV(PB0);

    while(1)
    { 
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

