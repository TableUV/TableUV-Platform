/**
 * @file dev_imu.h
 * @author Dong Jae (Alex) Park
 * @date 18 Feb 2021
 * @brief Device IMU
 * 
 * This document will contains device configure content
 */


#ifndef DEV_IMU_H
#define DEV_IMU_H
# ifdef __cplusplus
extern "C"{
# endif

// External Lib
#include <stdbool.h>

typedef enum {
    ACC_X,
    ACC_Y,
    ACC_Z,
    GYR_X,
    GYR_Y,
    GYR_Z,
    MAG_X,
    MAG_Y,
    MAG_Z,
    IMU_COUNT,
    IMU_UNDEFINED
} IMU;

void dev_imu_init(void);
bool dev_imu_get_values(IMU axis, float* data_ptr);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_IMU_H