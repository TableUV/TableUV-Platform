/**
 * @file dev_ToF_Lidar.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Array of ToF Lidars
 * 
 * This document will contains device configure content
 */


#ifndef DEV_TOF_LIDAR_H
#define DEV_TOF_LIDAR_H
# ifdef __cplusplus
extern "C"{
# endif 

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef enum{
    DEV_TOF_LIDAR_R,
    DEV_TOF_LIDAR_C,
    DEV_TOF_LIDAR_L,
    DEV_TOF_LIDAR_COUNT,
    DEV_TOF_LIDAR_UNKNOWN
} DEV_TOF_LIDAR_E;

///////////////////////////////////////
///////   PUBLIC PROTOTYPE    /////////
///////////////////////////////////////
void dev_ToF_Lidar_init(void);
void dev_ToF_Lidar_update(void);


# ifdef __cplusplus  
}
# endif 
#endif //DEV_TOF_LIDAR_H