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

#include <stdio.h>

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define ENABLE          (1U)
#define DISABLE         (0U)
#define TRUE            (1U)
#define FALSE           (0U)
#define TODO            (0U)

//////////////////////////////////////
///////   MODE DEFINITION     ////////
//////////////////////////////////////
#define PROJECT_MODE_PRODUCTION     (0U)
#define PROJECT_MODE_DEVELOPMENT    (1U)
#define PROJECT_MODE_SELECTION      (PROJECT_MODE_DEVELOPMENT)

/////////////////////////////////////////
///////   FEATURE SELECTION     ////////
/////////////////////////////////////////
/***************************************
 *****  => PRODUCTION SETTINGS  ********
 ***************************************/
#if (PROJECT_MODE_SELECTION == PROJECT_MODE_PRODUCTION)
#   define MOCK                                   (DISABLE)

/*****   FEATURE ENABLES  ****/
#   define FEATURE_LIDAR                   (ENABLE) // (WIP)
#   define FEATURE_LIDAR_CALIBRATION_MODE  (DISABLE) // TODO: implement calibration strategy
#   define FEATURE_LIDAR                          ( ENABLE)
#   define FEATURE_PERIPHERALS                    ( ENABLE)
#   define FEATURE_UV                             ( ENABLE)
#   define FEATURE_IMU                            ( ENABLE)
#   define FEATURE_SENSOR_AVR                     ( ENABLE)
#   define FEATURE_AVR_DRIVER_ALL                 ( ENABLE) //
#   ifndef FEATURE_AVR_DRIVER_ALL
#       define FEATURE_AVR_MOTOR                  ( ENABLE) //
#       define FEATURE_AVR_WATERLEVEL             ( ENABLE) //
#       define FEATURE_AVR_HAPTIC                 ( ENABLE) //
#       define FEATURE_AVR_ENCODER                ( ENABLE) //
#   endif // (FEATURE_AVR_DRIVER_ALL)

/*****   DEBUG PRINT FLAGS  ****/
#   define DEBUG_FPRINT                           (DISABLE)
#   ifdef DEBUG_FPRINT
#       define DEBUG_FPRINT_FEATURE_LIDAR         (DISABLE) // Live feed of sensor readings
#       define DEBUG_FPRINT_FEATURE_MAP           (DISABLE) 
#       define DEBUG_FPRINT_FEATURE_MAP_CENTERED  (  FALSE) // Mode: true->memory_map, false->center_map
#       define DEBUG_FPRINT_FEATURE_OBSTACLES     (DISABLE) // Live feed of obstacle detection
#   endif

/***************************************
 *****  => DEVELOPMENT SETTINGS  *******
 ***************************************/
#elif (PROJECT_MODE_SELECTION == PROJECT_MODE_DEVELOPMENT)
#   define MOCK                                   ( ENABLE)

/*****   FEATURE ENABLES  ****/
#   define FEATURE_DEV_DRIVER                     ( ENABLE) // DEV avr driver: motor, mist, encoder feedback
#   define FEATURE_SLAM                           ( ENABLE) // APP SLAM
#   define FEATURE_LIDAR                          ( ENABLE)
#   define FEATURE_SLAM_ENCODER                   ( ENABLE)
#   define FEATURE_DEMO_TOF_OBSTACLE        (FEATURE_LIDAR) // DEV avr driver: motor, mist, encoder feedback
#   define FEATURE_LIDAR_CALIBRATION_MODE         (   TODO) // TODO: implement calibration strategy
#   define FEATURE_SUPER_USE_PROFILED_MOTIONS     (   TODO) // TODO: implement calibration strategy
#   define FEATURE_SUPER_USE_HARDCODE_CHORE       ( ENABLE)
#   define FEATURE_SUPER_CMD_DEV_DRIVER           ( ENABLE) // Super command on actuators
#   define FEATURE_PERIPHERALS                    ( ENABLE)
#   define FEATURE_UV                             (DISABLE)
#   define FEATURE_IMU                            (DISABLE)
#   define FEATURE_SENSOR_AVR                     ( ENABLE)
#   define FEATURE_AVR_DRIVER_ALL                 ( ENABLE) //
#   ifndef FEATURE_AVR_DRIVER_ALL
#       define FEATURE_AVR_MOTOR                  ( ENABLE) //
#       define FEATURE_AVR_WATERLEVEL             ( ENABLE) //
#       define FEATURE_AVR_HAPTIC                 ( ENABLE) //
#       define FEATURE_AVR_ENCODER                ( ENABLE) //
#   endif // (FEATURE_AVR_DRIVER_ALL)
#   define FEATURE_SLAM_AVR_SENSOR           (FEATURE_SLAM)
#   define FEATURE_BATTERY                        ( ENABLE)

/*****   DEBUG PRINT FLAGS  ****/
#   define DEBUG_FPRINT                           ( ENABLE)
#   ifdef DEBUG_FPRINT
#       define DEBUG_FPRINT_FEATURE_AVR_DRIVER          ( ENABLE) // Live feed of of avr driver
#       define DEBUG_FPRINT_FEATURE_LIDAR               ( ENABLE) // Live feed of tof sensor readings
#       define DEBUG_FPRINT_APP_SLAM_PRINT              ( ENABLE) // Live feed of supervisor state
#       define DEBUG_FPRINT_APP_SUPER_STATE             ( ENABLE) // Live feed of supervisor state
#       define DEBUG_FPRINT_APP_SUPER_AVR_SENSOR        ( ENABLE) // Live feed of collision status
#       define DEBUG_FPRINT_APP_SUPER_CHOREOGRAPHY      ( ENABLE) // Live feed of choreography status
#       define DEBUG_FPRINT_FEATURE_MAP                 (DISABLE) // Live feed of global map
#       define DEBUG_FPRINT_FEATURE_MAP_CENTERED        (  FALSE) // Map Feeding Mode: true->memory_map, false->center_map
#       define DEBUG_FPRINT_FEATURE_OBSTACLES           (DISABLE) // Live feed of obstacle detection
#       define DEBUG_FPRINT_FEATURE_CHOREOGRAPHY        (DISABLE) // Live feed of choreography
#   else
#       define DEBUG_FPRINT_FEATURE_LIDAR               (DISABLE)
#       define DEBUG_FPRINT_APP_SUPER_STATE             (DISABLE)
#       define DEBUG_FPRINT_APP_SUPER_AVR_SENSOR        (DISABLE)
#       define DEBUG_FPRINT_APP_SUPER_CHOREOGRAPHY      (DISABLE)
#       define DEBUG_FPRINT_FEATURE_MAP                 (DISABLE)
#       define DEBUG_FPRINT_FEATURE_MAP_CENTERED        (DISABLE)
#       define DEBUG_FPRINT_FEATURE_OBSTACLES           (DISABLE)
#       define DEBUG_FPRINT_FEATURE_CHOREOGRAPHY        (DISABLE)
#   endif

/***********************************
 *****  => UNKNOWN SETTINGS  *******
 ***********************************/
#else
#   error ("Mode does not exist!")
#endif

///////////////////////////////////////////
///////   MACRO FUNC DEFINITION     ///////
///////////////////////////////////////////
#if (DEBUG_FPRINT)
#   define PRINTF(f_, ...) printf((f_), __VA_ARGS__)
#else
#   define PRINTF(f_, ...) // Do Nothing
#endif

#ifndef INLINE
#   if __GNUC__ && !__GNUC_STDC_INLINE__
#       define INLINE extern inline
#   else
#       define INLINE inline
#   endif
#endif

#define BYTE_TO_BINARY(byte)   \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0') 

#ifdef __cplusplus  
}
#endif 
#endif //COMMON_H