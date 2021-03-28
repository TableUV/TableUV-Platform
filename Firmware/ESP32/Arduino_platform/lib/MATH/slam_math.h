/**
 * @file math.h
 * @author Jianxiang (Jack) Xu
 * @date 02 March 2021
 * @brief Math header files
 *
 * This document will contain custom math
 */


#ifndef SLAM_MATH_H
#define SLAM_MATH_H
# ifdef __cplusplus
extern "C"{
# endif 

// std. C lib
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "dev_avr_driver.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    int32_t x;
    int32_t y;
} math_cart_coord_int32_S;

typedef struct{
    float x;
    float y;
} math_cart_coord_float_S;

///////////////////////////////////////
///////   PUBLIC PROTOTYPE    /////////
///////////////////////////////////////
/**
 * @brief Converts left and right encoder buffer into one x and y
 * 
 * @param l_enc_buf: Input left encoder buffer of size ENC_BUFFER_SIZE
 * @param r_enc_buf: Input right encoder buffer of size ENC_BUFFER_SIZE
 * @param enc_x:     Output pose x
 * @param enc_y:     Output pose y
 */
void slam_math_get_enc_pose(int16_t* l_enc_buf, int16_t* r_enc_buf, float* enc_x, float* enc_y) 
{
    float r_wheel_vel, l_wheel_vel, robot_vel, robot_theta;
    float total_x = 0;
    float total_y = 0;

    for(int i = 0; i ++; i < ENC_BUFFER_SIZE)
    {
        r_wheel_vel = r_enc_buf[i] * R_WHEEL_MM_PER_TICK * ENCODER_UPDATE_FREQ_HZ;
        l_wheel_vel = l_enc_buf[i] * L_WHEEL_MM_PER_TICK * ENCODER_UPDATE_FREQ_HZ;

        robot_vel = (r_wheel_vel + l_wheel_vel) * 0.5;
        robot_theta = (r_wheel_vel - l_wheel_vel) * 0.5 * INVERSE_DIST_BW_WHEELS_MM;

        total_x = total_x + robot_vel * cos(robot_theta);
        total_y = total_y + robot_vel * sin(robot_theta);
    }

    *enc_x = total_x;
    *enc_y = total_y;
}

# ifdef __cplusplus  
}
# endif 
#endif //MATH_H