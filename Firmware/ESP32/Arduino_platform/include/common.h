/**
 * @file common.h
 * @author Tsugmui Murata
 * @date 15 Feb 2021
 * @brief IO Ping Map Configuration
 *
 * This document will contains ping definitions
 */

#ifndef COMMON_H
#define COMMON_H
#ifdef __cplusplus
extern "C"{
#endif 

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define ENABLE          (1U)
#define DISABLE         (0U)
#define TRUE            (1U)
#define FALSE           (0U)

/////////////////////////////////////////
///////   FEATURE DEFINITION     ////////
/////////////////////////////////////////
#define FEATURE_LIDAR                   (ENABLE) // (WIP)
#define FEATURE_LIDAR_CALIBRATION_MODE  (DISABLE) // TODO: implement calibration strategy

#define DEBUG_FPRINT                    (DISABLE)

///////////////////////////////////////////
///////   MACRO FUNC DEFINITION     ///////
///////////////////////////////////////////
#if (DEBUG_FPRINT)
# define PRINTF(f_, ...) printf((f_), __VA_ARGS__)
#else
# define PRINTF(f_, ...) (void)((f_), __VA_ARGS__)
#endif

#ifdef __cplusplus  
}
#endif 
#endif //COMMON_H