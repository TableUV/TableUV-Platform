/**
 * @file math.c
 * @author Jianxiang (Jack) Xu
 * @date 02 March 2021
 * @brief Math algorithm
 *
 * This document will contains math algo.
 */

#include "slam_math.h"

// TableUV Lib

// External Lib

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////


/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////


///////////////////////////
///////   DATA     ////////
///////////////////////////


////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
math_cart_coord_float_S slam_math_get_enc_pose(int16_t* l_enc_buf, int16_t* r_enc_buf) 
{
    float r_wheel_vel, l_wheel_vel, robot_vel, robot_theta;
    math_cart_coord_float_S total_sum = {0};
    

    for (int i = 0; i ++; i < ENC_BUFFER_SIZE)
    {
        r_wheel_vel = r_enc_buf[i] * R_WHEEL_MM_PER_TICK * ENCODER_UPDATE_FREQ_HZ;
        l_wheel_vel = l_enc_buf[i] * L_WHEEL_MM_PER_TICK * ENCODER_UPDATE_FREQ_HZ;

        robot_vel = (r_wheel_vel + l_wheel_vel) * 0.5;
        robot_theta = (r_wheel_vel - l_wheel_vel) * 0.5 * INVERSE_DIST_BW_WHEELS_MM;

        total_sum.x = total_sum.x + robot_vel * cos(robot_theta);
        total_sum.y = total_sum.y + robot_vel * sin(robot_theta);
    }

    return total_sum;
}