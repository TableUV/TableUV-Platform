/**
 * @file dev_avr_driver.c
 * @author Tsugumi Murata (tmurata293)
 * @date 15 Feb 2021
 * @brief Device AVR Driver main file 
 *
 * This document will contains device configure content
 */

#include <Arduino.h>
#include "dev_avr_driver.h"
#include <Wire.h>
#include "../../include/avr_driver_common.h"
#include <stdbool.h>

#define I2C_RECIEVE_TIMEOUT_MILLI_SEC                                       10
#define MP_MUTEX_BLOCK_TIME_MS                                              ((1U)/portTICK_PERIOD_MS)

#define SET_MESSAGE_ESTOP_EN()                                              (1 << 13)
#define SET_MESSAGE_HAPTIC_EN()                                             (1 << 11)
#define SET_MESSAGE_TOF_CONFIG_EN(tof_sensor_config)                        (tof_sensor_config << 8)


#define RESET_MESSAGE_ESTOP_EN()                                            ~(1 << 13)
#define RESET_MESSAGE_HAPTIC_EN()                                           ~(1 << 11)
#define RESET_MESSAGE_TOF_CONFIG_EN(tof_sensor_config)                      ~(tof_sensor_config << 8)

typedef struct{
    TwoWire                     I2C;
    const uint8_t               address[NUM_AVR_DRIVER];
    const data_frame_header_E   dataFrameHeader[DATA_FRAME_HEADER_COUNT];
    uint16_t                    i2c_message[NUM_AVR_DRIVER];
    uint8_t                     reqEstop;
    uint8_t                     reqHaptic;
    tof_sensor_config_E         reqConfigTof;
    robot_motion_mode_E         reqRobotMotion;
    motor_pwm_duty_E            pwm_duty[NUM_AVR_DRIVER];
    uint16_t                    encoderCount[NUM_AVR_DRIVER];
    uint8_t                     waterLevelSig; 
    SemaphoreHandle_t           mp_mutex;
} dev_avr_driver_data_S;

///////////////////////////
///////   DATA     ////////
///////////////////////////

static dev_avr_driver_data_S dev_avr_driver_data = {
    .I2C     = TwoWire(0),
    .address = {
        LEFT_AVR_DRIVER_I2C_ADDRESS,
        RIGHT_AVR_DRIVER_I2C_ADDRESS
    },
    .dataFrameHeader = {
        DATA_FRAME_HEADER_FIRST,
        DATA_FRAME_HEADER_SECOND,
        DATA_FRAME_HEADER_THIRD,
        DATA_FRAME_HEADER_FOURTH
    },
    .i2c_message = {
        0x0000,
        0x0000
    },
    .reqEstop   = 1,
    .reqHaptic  = 0,
    .reqConfigTof = TOF_SENSOR_CONFIG_DISABLE_ALL,
    .reqRobotMotion = ROBOT_MOTION_BREAK,
    .pwm_duty  = {
        MOTOR_PWM_DUTY_0_PERCENT, 
        MOTOR_PWM_DUTY_0_PERCENT 
    },
    .encoderCount = {
        0,
        0
    },
    .waterLevelSig = 0,
    .mp_mutex = xSemaphoreCreateBinary()
};


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////

// I2C transmit two byte 
static void dev_avr_driver_transmit_two_byte(uint8_t address, uint16_t message){
    dev_avr_driver_data.I2C.beginTransmission(address);
    dev_avr_driver_data.I2C.write((message & DATA_MASK_16BIT_FIRST_8BIT) >> 8);
    dev_avr_driver_data.I2C.write( message & DATA_MASK_16BIT_SECOND_8BIT);
    dev_avr_driver_data.I2C.endTransmission(true);
}

// I2C receive one byte 
static uint8_t  dev_avr_driver_receive_one_byte(uint8_t address){
    uint8_t receive_first_byte;
    dev_avr_driver_data.I2C.requestFrom(address, 1, false); 
    receive_first_byte = dev_avr_driver_data.I2C.read(); 
    return receive_first_byte;
}

// I2C receive two byte 
static uint16_t dev_avr_driver_receive_two_byte(uint8_t address){
    uint8_t receive_first_byte, receive_second_byte; 
    dev_avr_driver_data.I2C.requestFrom(address, 2, true); 
    receive_first_byte = dev_avr_driver_data.I2C.read(); 
    receive_second_byte = dev_avr_driver_data.I2C.read();
    receive_second_byte = abs(receive_second_byte);
    return (receive_first_byte << 8) | receive_second_byte;
}

// initialize I2C message
static void dev_avr_driver_init_message_two_byte(){
    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]   = 0b0000000001000000;
    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER]  = 0b0000000001000000;
}

static void avr_driver_update_i2c_message_two_byte(){  
    dev_avr_driver_init_message_two_byte();
    dev_avr_driver_data_S * temp_dev_avr_driver_data; 
    if (xSemaphoreTake(dev_avr_driver_data.mp_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE){
        // memcpy data
        memcpy(temp_dev_avr_driver_data, &(dev_avr_driver_data), sizeof(dev_avr_driver_data_S));
        xSemaphoreGive(dev_avr_driver_data.mp_mutex); // release lock
    }
        // req estop 
        if( dev_avr_driver_data.reqEstop){
            dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= SET_MESSAGE_ESTOP_EN();
            dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= SET_MESSAGE_ESTOP_EN();
        }
        else{
            dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  &= RESET_MESSAGE_ESTOP_EN();
            dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] &= RESET_MESSAGE_ESTOP_EN();
        }

        // req haptic 
        if( dev_avr_driver_data.reqHaptic){
            dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= SET_MESSAGE_HAPTIC_EN();
        }
        else{
            dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] &= RESET_MESSAGE_HAPTIC_EN();
        }

        // req ConfigTof
        if( dev_avr_driver_data.reqConfigTof ){
            dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER] |= SET_MESSAGE_TOF_CONFIG_EN(dev_avr_driver_data.reqConfigTof);
        }
        else{
            dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER] &= (~(1 << 9) & ~(1 << 8)) ;
        }

        // req RobotMotion
        if (dev_avr_driver_data.reqRobotMotion){
            switch(dev_avr_driver_data.reqRobotMotion){

                case(ROBOT_MOTION_FW_COAST):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CCW << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                case(ROBOT_MOTION_REV_COAST):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CW  << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CCW << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                case(ROBOT_MOTION_FW_BREAK):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_BRAKE << 5) | (MOTOR_COMMAND_DIRECTION_CCW << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= (MOTOR_COMMAND_MODE_BRAKE << 5) | (MOTOR_COMMAND_DIRECTION_CW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                case(ROBOT_MOTION_REV_BREAK):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_BRAKE << 5) | (MOTOR_COMMAND_DIRECTION_CCW << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= (MOTOR_COMMAND_MODE_BRAKE << 5) | (MOTOR_COMMAND_DIRECTION_CW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                    break;

                case(ROBOT_MOTION_CW_ROTATION):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]   |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CW << 4)  | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                case(ROBOT_MOTION_CCW_ROTATION):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]   |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CCW  << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CCW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                case(ROBOT_MOTION_FW_DIFF_ROTATION):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CCW << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                case(ROBOT_MOTION_REV_DIFF_ROTATION):
                    dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CW   << 4) | (dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] << 0);
                    dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= (MOTOR_COMMAND_MODE_COAST << 5) | (MOTOR_COMMAND_DIRECTION_CCW  << 4) | (dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] << 0);
                break;

                default:
                break; 
            }     
        }
        else{
            dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER]  |= SET_MESSAGE_ESTOP_EN();
            dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] |= SET_MESSAGE_ESTOP_EN();
        }

    //    xSemaphoreGive(dev_avr_driver_data.mp_mutex); // release lock
    //}
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////

// init dev_avr_driver
void dev_avr_driver_init()
{
    dev_avr_driver_data.I2C.begin(MOTOR_I2C_SDA, MOTOR_I2C_SCL); 
    dev_avr_driver_init_message_two_byte(); 
    dev_avr_driver_set_timeout(I2C_RECIEVE_TIMEOUT_MILLI_SEC); 
    xSemaphoreGive(dev_avr_driver_data.mp_mutex);
}

void dev_driver_avr_update100ms()
{
    avr_driver_update_i2c_message_two_byte(); 
    dev_avr_driver_transmit_two_byte( dev_avr_driver_data.address[LEFT_AVR_DRIVER] , dev_avr_driver_data.i2c_message[LEFT_AVR_DRIVER] );
    dev_avr_driver_data.encoderCount[LEFT_AVR_DRIVER] = dev_avr_driver_receive_two_byte(dev_avr_driver_data.address[LEFT_AVR_DRIVER]);

    dev_avr_driver_transmit_two_byte( dev_avr_driver_data.address[RIGHT_AVR_DRIVER] , dev_avr_driver_data.i2c_message[RIGHT_AVR_DRIVER] );
    dev_avr_driver_data.encoderCount[RIGHT_AVR_DRIVER] = dev_avr_driver_receive_two_byte(dev_avr_driver_data.address[RIGHT_AVR_DRIVER]);
    //dev_avr_driver_data.waterLevelSig = dev_avr_driver_receive_one_byte(dev_avr_driver_data.address[RIGHT_AVR_DRIVER]);
    
    // take the mutex
    // if (xSemaphoreTake(dev_avr_driver_data.mp_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE) 
    // {
    // release the mutex 
    // xSemaphoreGive(dev_avr_driver_data.mp_mutex); 
}

void dev_avr_driver_set_timeout(uint8_t milliSec){
    dev_avr_driver_data.I2C.setTimeout(milliSec); 
}

void dev_avr_driver_set_req_Estop(){
    dev_avr_driver_data.reqEstop = 1;
}

void dev_avr_driver_set_req_Haptic(){
    dev_avr_driver_data.reqHaptic = 1;
}

void dev_avr_driver_set_req_Tof_config(tof_sensor_config_E tof_sensor_config){
    dev_avr_driver_data.reqConfigTof = tof_sensor_config; 
}
void dev_avr_driver_set_req_Robot_motion(robot_motion_mode_E robot_motion_mode, motor_pwm_duty_E motor_pwm_duty_left, motor_pwm_duty_E motor_pwm_duty_right){
    dev_avr_driver_reset_req_Estop();
    dev_avr_driver_data.reqRobotMotion = robot_motion_mode;
    dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] = motor_pwm_duty_left;
    dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] = motor_pwm_duty_right;
}

void dev_avr_driver_reset_req_Estop(){
    dev_avr_driver_data.reqEstop = 0;
}

void dev_avr_driver_reset_req_Haptic(){
    dev_avr_driver_data.reqHaptic = 0;
}

void dev_avr_driver_reset_req_Tof_config(){
    dev_avr_driver_data.reqConfigTof = TOF_SENSOR_CONFIG_DISABLE_ALL; 
}

void dev_avr_driver_reset_req_Robot_motion(){
    dev_avr_driver_set_req_Estop();
    dev_avr_driver_data.reqRobotMotion = ROBOT_MOTION_BREAK;
    dev_avr_driver_data.pwm_duty[LEFT_AVR_DRIVER] = MOTOR_PWM_DUTY_0_PERCENT;
    dev_avr_driver_data.pwm_duty[RIGHT_AVR_DRIVER] = MOTOR_PWM_DUTY_0_PERCENT;
}

uint16_t dev_avr_driver_get_EncoderCount(uint8_t driver_side){
    return dev_avr_driver_data.encoderCount[driver_side];
}

uint8_t  dev_avr_driver_get_WaterLevelSig(){
    return dev_avr_driver_data.waterLevelSig;
}