/**
 * @file dev_uv.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief UV LED Drivers
 *
 * This document contains implementations for UV
 */


#ifndef DEV_UV_H
#define DEV_UV_H
# ifdef __cplusplus
extern "C"{
# endif

#include <stdint.h>
#include <stdbool.h>

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef enum {
    DEV_UV_LED_ROW,
    DEV_UV_LED_SIDE,
    DEV_UV_LED_COUNT,
    DEV_UV_LED_UNKNOWN
} DEV_UV_E;

///////////////////////////////////////
///////   PUBLIC PROTOTYPE    /////////
///////////////////////////////////////

/**
 * @brief This function initializes the 
 */
void dev_uv_init(void);

/**
 * @brief This function sets both row and side UV LEDs on.
 * 
 * @param pwn_duty: buffer pointer used to copy over the data
 * @param dac_duty: buffer pointer used to copy over the data
 */
void dev_uv_set_both(int pwm_duty, uint8_t dac_duty);

/**
 * @brief This function sets row UV LEDs on.
 * 
 * @param pwn_duty: buffer pointer used to copy over the data
 * @param dac_duty: buffer pointer used to copy over the data
 */
void dev_uv_set_row(int pwm_duty, uint8_t dac_duty);

/**
 * @brief This function sets side UV LEDs on.
 * 
 * @param pwn_duty: buffer pointer used to copy over the data
 * @param dac_duty: buffer pointer used to copy over the data
 */
void dev_uv_set_side(int pwm_duty, uint8_t dac_duty);

/**
 * @brief This function turns both UV LEDs off.
 */
void dev_uv_stop();


# ifdef __cplusplus  
}
# endif 
#endif //DEV_UV_H