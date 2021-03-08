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
#   define FEATURE_LIDAR                          ( ENABLE)

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
#   define FEATURE_LIDAR                          (DISABLE)
#   define FEATURE_LIDAR_CALIBRATION_MODE         (   TODO) // TODO: implement calibration strategy
#   define FEATURE_SUPER_USE_PROFILED_MOTIONS     (   TODO) // TODO: implement calibration strategy

#   define FEATURE_AVR_DRIVER_ALL                 ( ENABLE) //
#   ifndef FEATURE_AVR_DRIVER_ALL
#       define FEATURE_AVR_MOTOR                  ( ENABLE) //
#       define FEATURE_AVR_WATERLEVEL             ( ENABLE) //
#       define FEATURE_AVR_HAPTIC                 ( ENABLE) //
#       define FEATURE_AVR_ENCODER                ( ENABLE) //
#   endif // (FEATURE_AVR_DRIVER_ALL)

/*****   DEBUG PRINT FLAGS  ****/
#   define DEBUG_FPRINT                           ( ENABLE)
#   ifdef DEBUG_FPRINT
#       define DEBUG_FPRINT_FEATURE_LIDAR         (DISABLE) // Live feed of sensor readings
#       define DEBUG_FPRINT_FEATURE_MAP           (DISABLE) 
#       define DEBUG_FPRINT_FEATURE_MAP_CENTERED  (  FALSE) // Mode: true->memory_map, false->center_map
#       define DEBUG_FPRINT_FEATURE_OBSTACLES     (DISABLE) // Live feed of obstacle detection
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

#ifdef __cplusplus  
}
#endif 
#endif //COMMON_H