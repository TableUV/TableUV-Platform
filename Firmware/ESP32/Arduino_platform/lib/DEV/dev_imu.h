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

void dev_imu_init(void);
float* dev_imu_run1000ms(float*, ICM_20948_SPI*)

# ifdef __cplusplus  
}
# endif 
#endif //DEV_LED_H