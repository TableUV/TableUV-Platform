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

// Motor Encoder Macros
#define DEV_AVR_DRIVER_R_WHEEL_MM_PER_TICK             (0.0185245F) //mm in terms of linear motion not angular
#define DEV_AVR_DRIVER_L_WHEEL_MM_PER_TICK             (0.01828202F)
// Since 1m: diff is 0.3 mm, hence we may assume they have same charateristics between left and right motor:
#define DEV_AVR_DRIVER_INVERSE_DIST_BW_WHEELS_MM       (0.01295606F)
// #define DEV_AVR_DRIVER_WHEEL_RADIUS                    (16.735F) //Not needed
// #define DEV_AVR_DRIVER_ENCODER_UPDATE_FREQ_HZ          (20U) // UNUSED ???
#define DEV_AVR_DRIVER_WHEEL_MM_PER_TICK_SCALED             (0.00920163F) // (0.5 * (AVG: 0.01840326))
#define DEV_AVR_DRIVER_INVERSE_DIST_BW_WHEELS_MM_SCALED     (0.0001192168704F) // (DEV_AVR_DRIVER_WHEEL_MM_PER_TICK_HALF * DEV_AVR_DRIVER_INVERSE_DIST_BW_WHEELS_MM)
#define DEV_AVR_DRIVER_ENC_BUFFER_SIZE                      (3U) // = 20Hz/10Hz + 1


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


void dev_avr_driver_set_timeout(uint8_t milliSec);
void dev_avr_driver_set_req_Estop();
void dev_avr_driver_set_req_Haptic();
void dev_avr_driver_set_req_Tof_config(tof_sensor_config_E tof_sensor_config);
void dev_avr_driver_set_req_Robot_motion(robot_motion_mode_E robot_motion_mode, motor_pwm_duty_E motor_pwm_duty_left, motor_pwm_duty_E motor_pwm_duty_right);

void dev_avr_driver_reset_req_Estop();
void dev_avr_driver_reset_req_Haptic();
void dev_avr_driver_reset_req_Tof_config();
void dev_avr_driver_reset_req_Robot_motion();

/**
 * @brief Accesses encoder value from specified motor
 * @param driver_side LEFT_AVR_DRIVER or RIGHT_AVR_DRIVER
 * @see avr_driver_common.h
 * @return 16 bit encoder value
 */
uint16_t dev_avr_driver_get_EncoderCount(uint8_t driver_side);
/**
 * @brief Accesses left and right encoder value buffers 
 * @param l_enc_buf Left encoder buffer of size DEV_AVR_DRIVER_ENC_BUFFER_SIZE
 * @param r_enc_buf Right encoder buffer of size DEV_AVR_DRIVER_ENC_BUFFER_SIZE
 * @return buffer size
 */
uint8_t dev_avr_driver_get_encoder_buffers(int16_t* l_enc_buf, int16_t* r_enc_buf);
uint8_t  dev_avr_driver_get_WaterLevelSig();

# ifdef __cplusplus  
}
# endif 
#endif //DEV_AVR_DRIVER_H