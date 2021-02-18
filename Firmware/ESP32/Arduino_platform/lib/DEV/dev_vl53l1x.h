/**
 * @file dev_vl53l1x.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Time-of-Flight sensor (vl53l1x)
 * 
 * Datasheet: https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * This document will contains device configure content
 */


#ifndef DEV_VL53L1X_H
#define DEV_VL53L1X_H
# ifdef __cplusplus
extern "C"{
# endif 

void dev_vl53l1x_init(void);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_VL53L1X_H