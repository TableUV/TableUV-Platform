/**
 * @file dev_avr_driver.h
 * @author Tsugumi Murata (tmurata293)
 * @date 15 Feb 2021
 * @brief Device AVR Driver header file 
 *
 * This document will contains device configure content
 */


#ifndef DEV_AVR_DRIVER_H
#define DEV_AVR_DRIVER_H
# ifdef __cplusplus
extern "C"{
# endif 

/////////////////////////////////
/////////   INCLUDE     /////////
/////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include "../IO/io_ping_map.h"
#include "../../include/avr_driver_common.h"


/////////////////////////////////
/////////   ENUM    /////////
/////////////////////////////////
typedef enum robot_motion_mode{
    ROBOT_MOTION_BREAK, 
    ROBOT_MOTION_FW_COAST,
    ROBOT_MOTION_REV_COAST,
    ROBOT_MOTION_FW_BREAK,
    ROBOT_MOTION_REV_BREAK,
    ROBOT_MOTION_CW_ROTATION,
    ROBOT_MOTION_CCW_ROTATION,
    ROBOT_MOTION_FW_DIFF_ROTATION,
    ROBOT_MOTION_REV_DIFF_ROTATION,
    ROBOT_MOTION_COUNT,
    ROBOT_MOTION_UNDEFINED
} robot_motion_mode_E; 


void dev_avr_driver_init();
void dev_driver_avr_update20ms();

/**
 * @brief Accesses encoder value from specified motor
 * @param driver_side LEFT_AVR_DRIVER or RIGHT_AVR_DRIVER
 * @see avr_driver_common.h
 * @return 16 bit encoder value
 */
uint16_t dev_avr_driver_get_EncoderCount(uint8_t driver_side);
uint8_t  dev_avr_driver_get_WaterLevelSig();

void dev_avr_driver_set_timeout(uint8_t milliSec);
void dev_avr_driver_set_req_Estop();
void dev_avr_driver_set_req_Haptic();
void dev_avr_driver_set_req_Tof_config(tof_sensor_config_E tof_sensor_config);
void dev_avr_driver_set_req_Robot_motion(robot_motion_mode_E robot_motion_mode, motor_pwm_duty_E motor_pwm_duty_left, motor_pwm_duty_E motor_pwm_duty_right);

void dev_avr_driver_reset_req_Estop();
void dev_avr_driver_reset_req_Haptic();
void dev_avr_driver_reset_req_Tof_config();
void dev_avr_driver_reset_req_Robot_motion();


# ifdef __cplusplus  
}
# endif 
#endif //DEV_AVR_DRIVER_H