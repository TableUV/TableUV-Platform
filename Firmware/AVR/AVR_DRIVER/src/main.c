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
#include "left_driver_peripherals.h"
#include "decodeI2C.h"

#define RESET_TIMEOUT_COUNT     1000

static uint8_t getDriverMode(){
    // check which driver avr it is 
    DDRB &= ~_BV(MODE_SELECT); 
    return (PINB & _BV(MODE_SELECT));
}

static void resetState(){
    eStopMotor(); 
    disableMistActuator();  
}

int main(void)
{
    // set system clock to 8MHz
    CLKPR  = _BV(CLKPCE);
    CLKPR  = 0;
    
    // define local variables 
    uint8_t driver_mode = 0; 
    uint8_t slave_address = 0x00;  

    //check mode pin 
    driver_mode = getDriverMode(); 

    //config for left driver 
    if (driver_mode) {
        slave_address = 0x0A;   //slave address for left driver avr 
        setup_TOF_XSHUT_Config();
        
    }
    //config for right driver 
    else{
        slave_address = 0x2A ;   //slave address for right driver avr 
        //setup only needed for right avr driver 
        setupWaterLevelConfig();
        setupMistActuatorConfig();
    }

    //setup usi twi settings 
    setupUsiTwiConfig();

    //setup encoder interrupt settings 
    setupEncoderConfig(); 
    //setup motor config 
    setupMotorConfig(PWM_MODE_PHASE_CORRECT); 


    //initialize the USI communicatin
    usiTwiSlaveInit(slave_address);
    char message_first_byte  = 0x00;
    char message_second_byte = 0x00;

    uint16_t time_out_count = 0; 

    while(1)
    { 
        

        // increment count when data not received
        time_out_count ++; 
        if (time_out_count > 50000){
            resetState(); 
            time_out_count = 0; 
        }

        //if data received from master
        if(usiTwiDataInReceiveBuffer()){

            // initialize message to 0 
            message_first_byte  = 0x00;
            message_second_byte = 0x00;
            time_out_count = 0; 

            // initialize message to 0 
            message_first_byte  = 0x00;
            message_second_byte = 0x00;
            time_out_count = 0; 

            // store the first byte of data 
            message_first_byte = usiTwiReceiveByte();

            // check if data is indeed the first data byte of message 
            if (checkDataHeader(message_first_byte, DATA_FRAME_HEADER_FIRST)){
                
                // check if eStop bit is enabled 
                if (message_first_byte & ESTOP_COMMAND_REQ_MASK){
                    eStopMotor(); 
                }
                //resume if eStop bit is not enabled 
                else{

                    // decode for left driver 
                    if(driver_mode){
                        uint8_t tof_config = message_first_byte & TOF_XSHUT_EN_REQ_BIT_MASK;
                        switch(tof_config){
                            case(TOF_SENSOR_CONFIG_DISABLE_ALL):
                                enable_TOF_XSHUT_All();
                            break;
                            case(TOF_CONFIG_1):
                                disable_TOF_XSHUT_1();
                            break;
                            case(TOF_CONFIG_2):
                                disable_TOF_XSHUT_2();
                            break;
                            case(TOF_CONFIG_3):
                                disable_TOF_XSHUT_3();
                            break;
                            default:
                            break;
                        } 
                    }
                    // decode for right detector 
                    else{
                        // check if haptic req bit is enabled 
                        // #define IS_HAPTICREQUESTED(fbyte) ((fbyte)|(HAPTIC_EN_REQ_MASK))
                        if (message_first_byte & HAPTIC_EN_REQ_MASK)    enableMistActuator(); 
                        else                                            disableMistActuator(); 

                    }

                    message_second_byte = usiTwiReceiveByte();
                    motor_command_mode_E motor_command_mode;
                    motor_command_direction_E motor_command_direction;
                    motor_pwm_duty_E motor_pwm_duty; 

                    // check if data is indeed the second data byte of message 
                    if (checkDataHeader(message_second_byte, DATA_FRAME_HEADER_SECOND)){
                        
                        // check motor command mode  
                        if (message_second_byte & MOTOR_MODE_REQ_MASK)      motor_command_mode = MOTOR_COMMAND_MODE_BRAKE;
                        else                                                motor_command_mode = MOTOR_COMMAND_MODE_COAST; 

                        // check motor command direction 
                        if (message_second_byte & MOTOR_DIRECTION_REQ_MASK) motor_command_direction = MOTOR_COMMAND_DIRECTION_CW; 
                        else                                                motor_command_direction = MOTOR_COMMAND_DIRECTION_CCW; 
                            
                        motor_pwm_duty = (message_second_byte & MOTOR_PWM_DUTY_REQ_MASK);

                        if (motor_command_direction == MOTOR_COMMAND_DIRECTION_CW){
                            if (motor_command_mode == MOTOR_COMMAND_MODE_COAST)           setMotor(MOTOR_MODE_CW_COAST, motor_pwm_duty);
                            else if (motor_command_mode == MOTOR_COMMAND_MODE_BRAKE)      setMotor(MOTOR_MODE_CW_BREAK, motor_pwm_duty);
                        }
                        else if (motor_command_direction == MOTOR_COMMAND_DIRECTION_CCW){
                            if (motor_command_mode == MOTOR_COMMAND_MODE_COAST)           setMotor(MOTOR_MODE_CCW_COAST, motor_pwm_duty);
                            else if (motor_command_mode == MOTOR_COMMAND_MODE_BRAKE)      setMotor(MOTOR_MODE_CCW_BREAK, motor_pwm_duty);
                        }
                    }
                }
                // send data back to master
                cli();

                // send encoder data 
                usiTwiTransmitByte(getEncoderCount16_first_8bit());
                usiTwiTransmitByte(getEncoderCount16_second_8bit());
                setEncoderCount(0);

                //send water level signal 
                //usiTwiTransmitByte(getWaterLevelSignal());
                sei();
            }     
        }
        

    }
    return 0;   /* never reached */
}

