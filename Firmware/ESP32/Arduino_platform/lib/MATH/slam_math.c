/**
 * @file math.c
 * @author Jianxiang (Jack) Xu
 * @date 02 March 2021
 * @brief Math algorithm
 *
 * This document will contains math algo.
 */

#include "slam_math.h"
#include "../../include/common.h"
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
math_cart_coord_float_S slam_math_get_enc_pose(int16_t* l_enc_buf, int16_t* r_enc_buf, const uint8_t buffer_size) 
{
    float r_wheel_disp, l_wheel_disp, robot_disp, robot_theta;
    math_cart_coord_float_S total_sum = {0, 0};

    for (uint8_t i = 0; i < buffer_size; i ++)
    {
        r_wheel_disp = r_enc_buf[i] * DEV_AVR_DRIVER_R_WHEEL_MM_PER_TICK;
        l_wheel_disp = l_enc_buf[i] * DEV_AVR_DRIVER_L_WHEEL_MM_PER_TICK;

        robot_disp = (r_wheel_disp + l_wheel_disp) * 0.5;
        robot_theta = (r_wheel_disp - l_wheel_disp) * 0.5 * DEV_AVR_DRIVER_INVERSE_DIST_BW_WHEELS_MM;
        total_sum.x = total_sum.x + robot_disp * cos(robot_theta);
        total_sum.y = total_sum.y + robot_disp * sin(robot_theta);
    }

    return total_sum;
}

math_cart_coord_float_S slam_math_get_enc_pose_optimized(int16_t* l_enc_buf, int16_t* r_enc_buf, const uint8_t buffer_size) 
{
    float robot_disp, robot_theta;
    int32_t delta_disp, sum_disp;
    float x = 0;
    float y = 0;

    for (uint8_t i = 0; i < buffer_size; i ++)
    {
        sum_disp    = (r_enc_buf[i] + l_enc_buf[i]);
        delta_disp  = (r_enc_buf[i] - l_enc_buf[i]);

        robot_disp = sum_disp * DEV_AVR_DRIVER_WHEEL_MM_PER_TICK_SCALED;
        robot_theta = delta_disp *  DEV_AVR_DRIVER_INVERSE_DIST_BW_WHEELS_MM_SCALED;

        x = x + robot_disp * cos(robot_theta);
        y = y + robot_disp * sin(robot_theta);
    }

    math_cart_coord_float_S total_sum = {x, y};
    return total_sum;
}

math_cart_coord_float_S slam_math_get_enc_pose_reduced(int16_t* l_enc_buf, int16_t* r_enc_buf, const uint8_t buffer_size) 
{
    float r_wheel_disp, l_wheel_disp, robot_disp, robot_theta;
    math_cart_coord_float_S total_sum;

    robot_disp = 0.0f;
    robot_theta = 0.0f;
    for (uint8_t i = 0; i < buffer_size; i ++)
    {
        r_wheel_disp = r_enc_buf[i] * DEV_AVR_DRIVER_R_WHEEL_MM_PER_TICK;
        l_wheel_disp = l_enc_buf[i] * DEV_AVR_DRIVER_L_WHEEL_MM_PER_TICK;

        robot_disp += (r_wheel_disp + l_wheel_disp) * 0.5;
        robot_theta += (r_wheel_disp - l_wheel_disp) * 0.5 * DEV_AVR_DRIVER_INVERSE_DIST_BW_WHEELS_MM;
    }

    total_sum.x = robot_disp * cos(robot_theta);
    total_sum.y = robot_disp * sin(robot_theta);
    return total_sum;
}

float slam_math_get_theta(math_cart_coord_float_S input_coord)
{
    return atan2f(input_coord.y, input_coord.x);
}