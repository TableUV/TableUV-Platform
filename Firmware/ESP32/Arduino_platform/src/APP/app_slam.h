/**
 * @file    app_slam.h
 * @author  Jianxiang (Jack) Xu
 * @date    17 Feb 2021
 * @brief   Device LED
 * 
 * This document will contains slam app
 */

#ifndef APP_SLAM_H
#define APP_SLAM_H
# ifdef __cplusplus
extern "C"{
# endif 

#include <stdint.h>
#include <stdbool.h>
/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////

///////////////////////////////////////
///////   PUBLIC PROTOTYPE    /////////
///////////////////////////////////////
void app_slam_init(void);
void app_slam_run100ms(void);
void app_slam_requestToResetMap(void);

/**
 * @brief get motion velocity
 * 
 * It would return the velocity inside the motion profile buffer.
 * 
 * If the out of buffer size, return the last velocity.
 * 
 * return frame_stamp
 */
uint8_t app_slam_getMotionVelocity(int8_t * left_motor_mm_s_50ms, int8_t * right_motor_mm_s_50ms, uint8_t frame_stamp);

# ifdef __cplusplus  
}
# endif 
#endif //APP_SLAM_H