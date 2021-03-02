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

# ifdef __cplusplus  
}
# endif 
#endif //MATH_H