/**
 * @file dev_config.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure header files
 *
 * This document will contains device configure content
 */


#ifndef DEV_CONFIG_H
#define DEV_CONFIG_H
# ifdef __cplusplus
extern "C"{
# endif 

void dev_init(void);
void dev_run20ms(void);
void dev_run50ms(void);
void dev_run1000ms(void);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_CONFIG_H