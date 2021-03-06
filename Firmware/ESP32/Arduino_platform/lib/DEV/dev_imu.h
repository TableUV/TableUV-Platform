/**
 * @file dev_imu.h
 * @author Dong Jae (Alex) Park
 * @date 18 Feb 2021
 * @brief Device IMU
 * 
 * This document is the header file for interfacing with the IMU.
 */


#ifndef DEV_IMU_H
#define DEV_IMU_H
# ifdef __cplusplus
extern "C"{
# endif

// External Lib
#include <stdbool.h>

typedef enum {
    IMU_AXIS_ACC_X,
    IMU_AXIS_ACC_Y,
    IMU_AXIS_ACC_Z,
    IMU_AXIS_GYR_X,
    IMU_AXIS_GYR_Y,
    IMU_AXIS_GYR_Z,
    IMU_AXIS_MAG_X,
    IMU_AXIS_MAG_Y,
    IMU_AXIS_MAG_Z,
    IMU_AXIS_IMU_COUNT,
    IMU_AXIS_IMU_UNDEFINED
} IMU_AXIS_E;

/**
 * @brief This function initializes the IMU by setting the SPI bus
 */
void dev_imu_init(void);


/**
 * @brief This function fetches data from the IMU
 * 
 *  NOTE: Expecting app level call this function once per XX Hz (XXXms)
 *  NOTE: Acceleration values are converted from [mg] to [m/s^2]
 * @param data_ptr: pointer used to copy over the data
 * 
 * @return boolean on whether getting values were successful or not
 */
bool dev_imu_get_values(float* data_ptr);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_IMU_H