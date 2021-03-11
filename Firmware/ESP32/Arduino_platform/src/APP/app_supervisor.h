/**
 * @file    app_supervisor.h
 * @author  Jianxiang (Jack) Xu
 * @date    17 Feb 2021
 * @brief   App level
 * 
 * This document will contains slam app
 */

#ifndef APP_SUPERVISOR_H
#define APP_SUPERVISOR_H
# ifdef __cplusplus
extern "C"{
# endif 

void app_supervisor_init(void);
void app_supervisor_run50ms(void);


# ifdef __cplusplus  
}
# endif 
#endif //APP_SUPERVISOR_H