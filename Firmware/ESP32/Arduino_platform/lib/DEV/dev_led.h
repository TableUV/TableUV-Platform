/**
 * @file dev_led.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device LED
 * 
 * This document will contains device configure content
 */


#ifndef DEV_LED_H
#define DEV_LED_H
# ifdef __cplusplus
extern "C"{
# endif 

#include <stdbool.h>

void dev_led_init(void);
void dev_button_update(void);
bool dev_button_get(void);
void dev_led_update(void);
void dev_led_green_set(bool led_on);
void dev_led_red_set(bool led_on);
void dev_led_orange_set(bool led_on);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_LED_H