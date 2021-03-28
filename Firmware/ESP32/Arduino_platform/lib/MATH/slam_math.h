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
 * @return returns resulting pose
 */
math_cart_coord_float_S slam_math_get_enc_pose(int16_t* l_enc_buf, int16_t* r_enc_buf);

float slam_math_get_theta(math_cart_coord_float_S input_coord);

# ifdef __cplusplus  
}
# endif 
#endif //MATH_H