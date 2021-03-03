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
#define FEATURE_LIDAR                   (ENABLE)  // (WIP)
#define FEATURE_LIDAR_CALIBRATION_MODE  (DISABLE) // TODO: implement calibration strategy

#define FEATURE_AVR_DRIVER_ALL          (ENABLE) //

#ifndef  FEATURE_AVR_DRIVER_ALL
#define FEATURE_AVR_MOTOR               (ENABLE) //
#define FEATURE_AVR_WATERLEVEL          (ENABLE) //
#define FEATURE_AVR_HAPTIC              (ENABLE) //
#define FEATURE_AVR_ENCODER             (ENABLE) //
#endif 

#define DEBUG_FPRINT                    (ENABLE)
#define DEBUG_FPRINT_FEATURE_LIDAR      (DISABLE) // Live feed of sensor readings
#define DEBUG_FPRINT_FEATURE_MAP        (ENABLE)

///////////////////////////////////////////
///////   MACRO FUNC DEFINITION     ///////
///////////////////////////////////////////
#if (DEBUG_FPRINT)
# define PRINTF(f_, ...) printf((f_), __VA_ARGS__)
#else
# define PRINTF(f_, ...) (void)((f_), __VA_ARGS__)
#endif

#ifndef INLINE
# if __GNUC__ && !__GNUC_STDC_INLINE__
#  define INLINE extern inline
# else
#  define INLINE inline
# endif
#endif

#ifdef __cplusplus  
}
#endif 
#endif //COMMON_H