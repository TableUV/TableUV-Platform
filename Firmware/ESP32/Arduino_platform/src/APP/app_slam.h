/**
 * @file    app_slam.h
 * @author  Jianxiang (Jack) Xu
 * @date    17 Feb 2021
 * @brief   Device LED
 * 
 * This document will contains slam app
 */

#ifndef APP_SLAM_H
#define APP_SLAM_H
# ifdef __cplusplus
extern "C"{
# endif 

void app_slam_init(void);
void app_slam_run50ms(void);

# ifdef __cplusplus  
}
# endif 
#endif //APP_SLAM_H