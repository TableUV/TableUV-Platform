/**
 * @file dev_ToF_Lidar.h
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Array of ToF Lidars
 * 
 * This file contains the header file of ToF Lidar.
 */


#ifndef DEV_TOF_LIDAR_H
#define DEV_TOF_LIDAR_H
# ifdef __cplusplus
extern "C"{
# endif 

#include <stdint.h>
#include <stdbool.h>
/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef enum{
    DEV_TOF_FIRING_KEYFRAME_0 = (0U),
    DEV_TOF_FIRING_KEYFRAME_1,
    DEV_TOF_FIRING_KEYFRAME_2,
    DEV_TOF_FIRING_KEYFRAME_3,
    DEV_TOF_FIRING_KEYFRAME_4,
    DEV_TOF_FIRING_KEYFRAME_COUNT,
    DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
} DEV_TOF_FIRING_KEYFRAME_E;

typedef enum{
    DEV_TOF_LIDAR_C = (0U),
    DEV_TOF_LIDAR_L,
    DEV_TOF_LIDAR_R,
    DEV_TOF_LIDAR_COUNT,
    DEV_TOF_LIDAR_UNKNOWN
} DEV_TOF_LIDAR_E;

typedef enum{ // Geometrical Region 1 -> 15
    DEV_TOF_FIRING_GEOMETRICAL_1 = (0U),
    DEV_TOF_FIRING_GEOMETRICAL_2,
    DEV_TOF_FIRING_GEOMETRICAL_3,
    DEV_TOF_FIRING_GEOMETRICAL_4,
    DEV_TOF_FIRING_GEOMETRICAL_5,
    DEV_TOF_FIRING_GEOMETRICAL_6,
    DEV_TOF_FIRING_GEOMETRICAL_7,
    DEV_TOF_FIRING_GEOMETRICAL_8,
    DEV_TOF_FIRING_GEOMETRICAL_9,
    DEV_TOF_FIRING_GEOMETRICAL_10,
    DEV_TOF_FIRING_GEOMETRICAL_11,
    DEV_TOF_FIRING_GEOMETRICAL_12,
    DEV_TOF_FIRING_GEOMETRICAL_13,
    DEV_TOF_FIRING_GEOMETRICAL_14,
    DEV_TOF_FIRING_GEOMETRICAL_15,
    DEV_TOF_FIRING_GEOMETRICAL_COUNT,
    DEV_TOF_FIRING_GEOMETRICAL_UNKNOWN,
} DEV_TOF_FIRING_GEOMETRICAL_REGION_E;

#define DEV_TOF_TOTAL_POINTS_PER_SCAN       (DEV_TOF_LIDAR_COUNT * DEV_TOF_FIRING_KEYFRAME_COUNT)
#define DEV_TOF_BUFFER_SCALE                (2U) // 2x buffer shall be enough to cover
#define DEV_TOF_BUFFER_SIZE                 (DEV_TOF_TOTAL_POINTS_PER_SCAN * DEV_TOF_BUFFER_SCALE)

typedef struct {
    uint16_t    dist_mm[DEV_TOF_BUFFER_SIZE];        // data
    uint8_t     keyframe_label[DEV_TOF_BUFFER_SIZE]; // Geo Label
    uint8_t     data_counter;                        // counter
} dev_tof_lidar_sensor_data_S;
///////////////////////////////////////
///////   PUBLIC PROTOTYPE    /////////
///////////////////////////////////////
/**
 * @brief This function initialize I2C and GPIO interfaces for all listed Lidars
 */
void dev_ToF_Lidar_init(void);

/**
 * @brief This function updates data & firing sequence per 20ms
 */
void dev_ToF_Lidar_update20ms(void);

/**
 * @brief This function fetches data and reset the tof buffer
 * 
 *  NOTE: Expecting to call this function before the buffer fills up, else suffer buffer overridden
 *  NOTE: Expecting app level call this function once per 10 Hz (100ms)
 * @param buffer: buffer pointer used to copy over the data
 * 
 * @return status of the action
 */
bool dev_ToF_Lidar_dampDataBuffer(dev_tof_lidar_sensor_data_S* buffer);

# ifdef __cplusplus  
}
# endif 
#endif //DEV_TOF_LIDAR_H