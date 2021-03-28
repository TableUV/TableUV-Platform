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

/**
 *  @brief There are five range statuses: 0, 1, 2, 4, and 7. 
 *       Range status is 0      => no error. 
 *       Range status 1 and 2   => error warnings
 *       Range status 4 and 7   => errors.
 *  @details Range status:
 *          1: (sigma failure) => repeatability or standard deviation of the measurement is bad due to a decreasing signal noise ratio. 
 *              Increasing the timing budget can improve the standard deviation and avoid a range status 1.
 *          2: (signal failure)=> This means that the return signal is too week to return a good answer. 
 *             The reason is because the target is too far, or the target is not reflective enough, or the target is too small. 
 *              Increasing the timing budget might help, but there may simply be no target available.
 *          4: (out of bounds)=> This means that the sensor is ranging in a “nonappropriated” zone and the measured result may be inconsistent.
 *             This status is considered as a warning but, in general, it happens when a target is at the maximum distance possible from the sensor, i.e. around 5 m. 
 *              However, this is only for very bright targets.
 *          7: (wraparound)=> This situation may occur when the target is very reflective and the distance to the target/sensor is longer than the physical limited distance measurable by the sensor.
 *             Such distances include approximately 5 m when the senor is in Long distance mode and approximately 1.3 m when the sensor is in Short distance mode.
 *              Example: a traffic sign located at 6 m can be seen by the sensor and returns a range of 1 m. This is due to “radar aliasing”: 
 *              if only an approximate distance is required, we may add 6 m to the distance returned. However, that is a very approximate estimation.
 */

typedef enum{
    DEV_TOF_RANGE_STATUS_NO_ERROR               = 0,
    DEV_TOF_RANGE_STATUS_SIGMA_FAILURE          = 1,
    DEV_TOF_RANGE_STATUS_SIGNAL_FAILURE         = 2,
    DEV_TOF_RANGE_STATUS_OUT_OF_BOUNDS_FAILURE  = 4,
    DEV_TOF_RANGE_STATUS_WRAPAROUND_FAILURE     = 7,
} DEV_TOF_RANGE_STATUS_E;

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