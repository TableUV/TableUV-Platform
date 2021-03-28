/**
 * @file    app_slam.c
 * @author  Jianxiang (Jack) Xu
 * @date    15 Feb 2021
 * @brief   Device configure files
 *
 * This document will contains slam content
 */

#include "app_slam.h"

// Std. Lib
#include <stdio.h>
#include <string.h>

// TableUV Lib
#include "common.h"
#include "dev_ToF_Lidar.h"
#include "slam_math.h"
#include "dev_avr_sensor.h"
#include "dev_avr_driver.h"

// SDK config 
#include "sdkconfig.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//////////////////////////////
///////   TYPEDEF     ////////
//////////////////////////////
typedef int8_t map_pixel_data_t; // [-128, 127]
/* === === [ Vehicle Edge Node ] === ===
 * (0,0) ----->
 *   |             %%%  14  %%%
 *            %%%              %%%
 *      
 *        %%%                      %%%
 *      
 *       %%%                         %%%
 *                           x
 *  (R)   21            _____         7   (L)
 *                     |
 *       %%%         / | \           %%%
 *          (- theta)  y (+ theta)
 *          %%%     theta=0      %%%
 *      
 *               %%%  28|0  %%%
 *                   (FRONT)
 */

typedef enum {
    VEHICLE_EDGE_NODE_0,
    VEHICLE_EDGE_NODE_1,
    VEHICLE_EDGE_NODE_2,
    VEHICLE_EDGE_NODE_3,
    VEHICLE_EDGE_NODE_4,
    VEHICLE_EDGE_NODE_5,
    VEHICLE_EDGE_NODE_6,
    VEHICLE_EDGE_NODE_7,
    VEHICLE_EDGE_NODE_8,
    VEHICLE_EDGE_NODE_9,
    VEHICLE_EDGE_NODE_10,
    VEHICLE_EDGE_NODE_11,
    VEHICLE_EDGE_NODE_12,
    VEHICLE_EDGE_NODE_13,
    VEHICLE_EDGE_NODE_14,
    VEHICLE_EDGE_NODE_15,
    VEHICLE_EDGE_NODE_16,
    VEHICLE_EDGE_NODE_17,
    VEHICLE_EDGE_NODE_18,
    VEHICLE_EDGE_NODE_19,
    VEHICLE_EDGE_NODE_20,
    VEHICLE_EDGE_NODE_21,
    VEHICLE_EDGE_NODE_22,
    VEHICLE_EDGE_NODE_23,
    VEHICLE_EDGE_NODE_24,
    VEHICLE_EDGE_NODE_25,
    VEHICLE_EDGE_NODE_26,
    VEHICLE_EDGE_NODE_27,
    VEHICLE_EDGE_NODE_COUNT,
    VEHICLE_EDGE_NODE_UNKNOWN,
} vehicle_edge_node_E;

typedef enum {
    IR_RR,
    IR_RF,
    IR_FR,
    IR_FL,
    IR_LF,
    IR_LR,
    IR_COUNT,
    IR_UNKNOWN
} vehicle_IR_channel_E;

typedef enum {
    COLLISION_R,
    COLLISION_L,
    COLLISION_COUNT,
    COLLISION_UNKNOWN
} vehicle_collision_channel_E;

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
// Edge node mapping:
#define VEHICLE_EDGE_NODE_PI                (VEHICLE_EDGE_NODE_14)
#define VEHICLE_EDGE_NODE_FOV               (0.2243994753F)//(float)((CONST_M_PI) / (int32_t)(VEHICLE_EDGE_NODE_PI))
#define VEHICLE_EDGE_NODE_FOV_2             (0.1121997376F)//(float)(VEHICLE_EDGE_NODE_FOV/2)
#define VEHICLE_COLLISION_START_NODE        (VEHICLE_EDGE_NODE_0)
#define VEHICLE_COLLISION_NUM_NODES         (5)
#define VEHICLE_AVOIDANCE_R_PIXEL           (4)

// TOF: 
#if (FEATURE_DEMO_TOF_OBSTACLE)
// obstacle
# define VEHICLE_TOF_OBSTACLE_DIST_MIN           (30U) // [mm]
# define VEHICLE_TOF_OBSTACLE_DIST_MIN_CORNER    (10U) // [mm]
#endif // (FEATURE_DEMO_TOF_OBSTACLE)

/*** (parameterization) ***/
// Robot Characteristics 
#define ROBOT_SIZE_D_MM                     (100U)  // 100 [mm] => boundary would be (100 + 10/2 + 10/2) = 110 [mm]
// Global Map
#define GMAP_SQUARE_EDGE_SIZE_MM            (1000U) // 1   [m]
#define GMAP_UNIT_GRID_STEP_SIZE_MM         (10U)   // 10  [mm]

// Grid Occupancy
#define GRID_CELL_NEUTRAL                   (0x0) // <- have to be 0 (Unexplored score)
// Choose Rating between (-1 ~ -50) 
//      minimum: (GRID_CELL_WALKABLE_THRESHOLD_MIN)
//      : regularization term for path planning, 
//        -> -50 => less likely to walk through repetitive path
#define GRID_CELL_VISITED_SENSOR            (-40)
#define GRID_CELL_VISITED                   (-30)

// Rating in: 0~100 : ToF | MAX for collision switch
#define GRID_CELL_OCCUPANCY_MAX_PROB        (100)
// Rating in: 1~20 + 100 => must not intrude!
#define GRID_CELL_EDGE_MIN_PROB             (101)
#define GRID_CELL_EDGE_DEFAULT_PROB         (110)
#define GRID_CELL_EDGE_MAX_PROB             (120)

// 20 <= val <= 20 : walkable
#define GRID_CELL_WALKABLE_THRESHOLD_MAX    (20)
#define GRID_CELL_WALKABLE_THRESHOLD_MIN    (-50)

// grid cell parameterization
#define GRID_CELL_ALPHA_DECAY               (50) // \in [0, 100] : ->100 more weighted on new cell value (multiplicative decay)
#define GRID_CELL_ALPHA_DECAY_BASE          (100)
#define GRID_CELL_BETA_DECAY                (-3) // additive decay

// Obstacle avoidance
#define OBSTACLE_TOLERANCE                  (0U) // 0 tolerance
#define VELOCITY_BUFFER_SIZE                (4U)
#define VELOCITY_ZERO_MM_S                   (0) // 0 mm/s
#define VELOCITY_MIN_MM_S                   (10) // 10 mm/s
#define VELOCITY_SOFT_MM_S                  (30) // 30 mm/s
#define VELOCITY_MAX_MM_S                   (60) // 60 mm/s

#define MP_MUTEX_BLOCK_TIME_MS              ((1U)/portTICK_PERIOD_MS)

/*** (Pre-compile const.) ***/
// Robot Characteristics 
#define ROBOT_SIZE_D_PIXEL                  ((ROBOT_SIZE_D_MM)/(GMAP_UNIT_GRID_STEP_SIZE_MM))
#define ROBOT_SIZE_R_PIXEL                  ((ROBOT_SIZE_D_PIXEL)/(2U))
// Global Map 
#define GMAP_GRID_EDGE_SIZE_PIXEL           ((GMAP_SQUARE_EDGE_SIZE_MM)/(GMAP_UNIT_GRID_STEP_SIZE_MM))
#define GMAP_WN_PIXEL                       ((GMAP_GRID_EDGE_SIZE_PIXEL) + (1U))
#define GMAP_HN_PIXEL                       ((GMAP_GRID_EDGE_SIZE_PIXEL) + (1U))
#define GMAP_DEFAULT_CENTRAL_X_INDEX_PIXEL  ((GMAP_GRID_EDGE_SIZE_PIXEL) / (2U))
#define GMAP_DEFAULT_CENTRAL_Y_INDEX_PIXEL  ((GMAP_GRID_EDGE_SIZE_PIXEL) / (2U))
#define GMAP_VISIBILITY_RANGE_MAX           ((GMAP_SQUARE_EDGE_SIZE_MM)  / (2U))
// Others
#define CONST_M_2PI                         (6.283185307179586F)
#define CONST_M_PI		                    (3.14159265358979323846F)

/*** (Macro Functions) ***/
// Unit Conversion
#define GMAP_MM_TO_UNIT_PIXEL(x_mm)             (int32_t)((x_mm)/(GMAP_UNIT_GRID_STEP_SIZE_MM)) // -ve Ceiling => -1, +ve Flooring => 1
#define GMAP_UNIT_PIXEL_TO_MM(x_pixel)          (float)((x_pixel) * (float)(GMAP_UNIT_GRID_STEP_SIZE_MM))
#define ANGLE_WRAP_NPI_TO_PI(ang_rad)           (((ang_rad) < (- CONST_M_PI)?((CONST_M_2PI) + (ang_rad)):(((ang_rad) > (CONST_M_PI))?((ang_rad) - (CONST_M_2PI)):(ang_rad))))

// GMap dynamic accessor compensator
#define ARG_RANGE_INCLUSIVE(x_val, min, max)    (uint8_t)(((int32_t)(x_val) >= (int32_t)(min)) + ((int32_t)(x_val) >= (int32_t)(max))) // 0: (-inf, min), 1: [min, max], 2: [max, inf)

// update function for gmap (Exponentially weighted average)
#define GRID_CELL_UPDATE(old_val, new_val)      (map_pixel_data_t)((int32_t)((GRID_CELL_ALPHA_DECAY) * (int32_t)(new_val) + (GRID_CELL_ALPHA_DECAY_BASE - GRID_CELL_ALPHA_DECAY) * (int32_t)(old_val))/(GRID_CELL_ALPHA_DECAY_BASE))
#define GRID_CELL_MAX_SATURATION(value)         (map_pixel_data_t)(((value) <= (GRID_CELL_EDGE_MAX_PROB))?(value):(GRID_CELL_EDGE_MAX_PROB))
#define GRID_CELL_DECAY(value)                  (map_pixel_data_t)(((value) <= (GRID_CELL_NEUTRAL))?(value):((value) + (GRID_CELL_BETA_DECAY)))

#define EDGE_NODE_MAPPING(theta_rad)            (vehicle_edge_node_E)(((theta_rad) + (CONST_M_PI) + (VEHICLE_EDGE_NODE_FOV_2)) / (VEHICLE_EDGE_NODE_FOV)) // Assume: theta \in [-pi, pi]
#define EDGE_NODE_WRAPPING(node_integer)        (vehicle_edge_node_E)( ((node_integer) < 0) ? ((node_integer) + VEHICLE_EDGE_NODE_COUNT) : ( ((node_integer) >= (int8_t)(VEHICLE_EDGE_NODE_COUNT))?((node_integer) - VEHICLE_EDGE_NODE_COUNT):(node_integer) ) )

// Assumptions:
#if !(GMAP_WN_PIXEL == GMAP_HN_PIXEL)
    #error "GMAP_WN_PIXEL != GMAP_HN_PIXEL"
#endif

/* === === [ Global Grid Occupancy Map ] === ===
 *
 *      +------+----- WN = (E + 1) ------+
 *      |      |  0    ... E/2  ...    E |
 *      +------+-------------------------+
 *      | 0    |                         |
 *      | .    |                         |
 *      | .    |                         |
 *      | E/2  |          (0,0)          |   HN = (E + 1)
 *      | .    |                         |
 *      | .    |                         |
 *      | E    |                         |
 *      +------+-------------------------+
 */
typedef struct{
    map_pixel_data_t             data[GMAP_WN_PIXEL * GMAP_HN_PIXEL];
    math_cart_coord_int32_S      map_center_pixel;     // \in [0, GMAP_GRID_EDGE_SIZE_PIXEL]
    math_cart_coord_float_S      map_offset_mm;        // \in [-GMAP_UNIT_GRID_STEP_SIZE_MM, GMAP_UNIT_GRID_STEP_SIZE_MM]
    float                        vehicle_orientation_rad;
    vehicle_edge_node_E          orientation_node;
} dynamic_map_S;

typedef struct{
    const int8_t                edge_node_x_pixel[VEHICLE_EDGE_NODE_COUNT];
    const int8_t                edge_node_y_pixel[VEHICLE_EDGE_NODE_COUNT];
    const vehicle_edge_node_E   edge_node_ir[IR_COUNT];
} edge_sensor_config_S;

typedef struct {
    int8_t                      left_motor_velocity_mm_s_per_50ms[VELOCITY_BUFFER_SIZE];
    int8_t                      right_motor_velocity_mm_s_per_50ms[VELOCITY_BUFFER_SIZE];
    uint8_t                     update_stamp_100ms;
    uint8_t                     read_index;
} motion_profile_S;

typedef struct{
    // data:
    dev_tof_lidar_sensor_data_S lidar_data;
    bool                        ir_node[IR_COUNT];
    bool                        collision_end_node[COLLISION_COUNT];
    uint8_t                     obstacle_count; // store obstacle result
    uint16_t                    tof_dangerous_zone;

    SemaphoreHandle_t           motion_profile_mutex;
    motion_profile_S            motion_profile;

    // interface
    bool                        mapResetRequested;

    // global map info.
    dynamic_map_S               gMap;

    // sensor configuration
    const edge_sensor_config_S * sensor_config;

    int16_t left_enc_buf[ENC_BUFFER_SIZE];
    int16_t right_enc_buf[ENC_BUFFER_SIZE];

    math_cart_coord_float_S     encoder_delta_mm;
} app_slam_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static void app_slam_private_localization(void);
static void app_slam_private_localMapUpdate(void);
static void app_slam_private_globalMapUpdate(void);
static void app_slam_private_obstacleDetection(void);
static void app_slam_private_pathPlanning(void);
static void app_slam_private_motionPlanning(void);

static INLINE void app_slam_private_resetGlobalMap(void);
static void app_slam_private_translateGlobalMap(int32_t dx, int32_t dy);
static void app_slam_private_clearVehicleRegion(void);
static void app_slam_private_updateEdgeRegion(void);

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_slam_data_S slam_data;

// global offset data
static const int32_t MAP_OFFSET[3] = {GMAP_WN_PIXEL, 0, -GMAP_WN_PIXEL};

// sensor config
static const edge_sensor_config_S edge_sensor_config = {
    .edge_node_x_pixel = {
        [VEHICLE_EDGE_NODE_0 ] =  0, // Front
        [VEHICLE_EDGE_NODE_1 ] =  1,
        [VEHICLE_EDGE_NODE_2 ] =  2,
        [VEHICLE_EDGE_NODE_3 ] =  3,
        [VEHICLE_EDGE_NODE_4 ] =  4,
        [VEHICLE_EDGE_NODE_5 ] =  4,
        [VEHICLE_EDGE_NODE_6 ] =  5,
        [VEHICLE_EDGE_NODE_7 ] =  5, // Left
        [VEHICLE_EDGE_NODE_8 ] =  5,
        [VEHICLE_EDGE_NODE_9 ] =  4,
        [VEHICLE_EDGE_NODE_10] =  4,
        [VEHICLE_EDGE_NODE_11] =  3,
        [VEHICLE_EDGE_NODE_12] =  2,
        [VEHICLE_EDGE_NODE_13] =  1,
        [VEHICLE_EDGE_NODE_14] =  0, // Rear
        [VEHICLE_EDGE_NODE_15] = -1,
        [VEHICLE_EDGE_NODE_16] = -2,
        [VEHICLE_EDGE_NODE_17] = -3,
        [VEHICLE_EDGE_NODE_18] = -4,
        [VEHICLE_EDGE_NODE_19] = -4,
        [VEHICLE_EDGE_NODE_20] = -5,
        [VEHICLE_EDGE_NODE_21] = -5, // Right
        [VEHICLE_EDGE_NODE_22] = -5,
        [VEHICLE_EDGE_NODE_23] = -4,
        [VEHICLE_EDGE_NODE_24] = -4,
        [VEHICLE_EDGE_NODE_25] = -3,
        [VEHICLE_EDGE_NODE_26] = -2,
        [VEHICLE_EDGE_NODE_27] = -1,
    },
    .edge_node_y_pixel = {
        [VEHICLE_EDGE_NODE_0 ] = -5, // Front
        [VEHICLE_EDGE_NODE_1 ] = -5,
        [VEHICLE_EDGE_NODE_2 ] = -4,
        [VEHICLE_EDGE_NODE_3 ] = -4,
        [VEHICLE_EDGE_NODE_4 ] = -3,
        [VEHICLE_EDGE_NODE_5 ] = -2,
        [VEHICLE_EDGE_NODE_6 ] = -1,
        [VEHICLE_EDGE_NODE_7 ] =  0, // Left
        [VEHICLE_EDGE_NODE_8 ] =  1,
        [VEHICLE_EDGE_NODE_9 ] =  2,
        [VEHICLE_EDGE_NODE_10] =  3,
        [VEHICLE_EDGE_NODE_11] =  4,
        [VEHICLE_EDGE_NODE_12] =  4,
        [VEHICLE_EDGE_NODE_13] =  5,
        [VEHICLE_EDGE_NODE_14] =  5, // Rear
        [VEHICLE_EDGE_NODE_15] =  5,
        [VEHICLE_EDGE_NODE_16] =  4,
        [VEHICLE_EDGE_NODE_17] =  4,
        [VEHICLE_EDGE_NODE_18] =  3,
        [VEHICLE_EDGE_NODE_19] =  2,
        [VEHICLE_EDGE_NODE_20] =  1,
        [VEHICLE_EDGE_NODE_21] =  0, // Right
        [VEHICLE_EDGE_NODE_22] = -1,
        [VEHICLE_EDGE_NODE_23] = -2,
        [VEHICLE_EDGE_NODE_24] = -3,
        [VEHICLE_EDGE_NODE_25] = -4,
        [VEHICLE_EDGE_NODE_26] = -4,
        [VEHICLE_EDGE_NODE_27] = -5,
    },
    .edge_node_ir = {
        [IR_RR] = VEHICLE_EDGE_NODE_20,
        [IR_RF] = VEHICLE_EDGE_NODE_22,
        [IR_FR] = VEHICLE_EDGE_NODE_27,
        [IR_FL] = VEHICLE_EDGE_NODE_1,
        [IR_LF] = VEHICLE_EDGE_NODE_6,
        [IR_LR] = VEHICLE_EDGE_NODE_8,
    },
};
////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static void app_slam_private_localization(void)
{
    // TODO: intake  IMU, Encoder => EKF
    
}

/**
 * @brief Feature Map Update
 * 
 * STEP:
 *      1. grab tof data from 'dev_ToF_Lidar'
 *      2. grab IR + Collision Data
 *      3. process data
 */
static void app_slam_private_localMapUpdate(void)
{
    // 1. Grab data from Lidar
#if (FEATURE_LIDAR)
    const dev_tof_lidar_sensor_data_S * lidar_data = &(slam_data.lidar_data);
    const bool status = dev_ToF_Lidar_dampDataBuffer(lidar_data);
#endif //(FEATURE_LIDAR)

    // 2. Grab IR + Collision Data
#if (FEATURE_SLAM_AVR_SENSOR)
    uint8_t avr_sensor_data = dev_avr_sensor_uart_get();
    bool* ir_node = slam_data.ir_node;
    bool* cn_node = slam_data.collision_end_node;
    // Interpret:
    ir_node[IR_RR]       = (avr_sensor_data) & (DEV_AVR_RIGHT_IR_1);
    ir_node[IR_RF]       = (avr_sensor_data) & (DEV_AVR_RIGHT_IR_2);
    ir_node[IR_FR]       = (avr_sensor_data) & (DEV_AVR_FRONT_IR_1);
    ir_node[IR_FL]       = (avr_sensor_data) & (DEV_AVR_FRONT_IR_2);
    ir_node[IR_LF]       = (avr_sensor_data) & (DEV_AVR_LEFT_IR_2);
    ir_node[IR_LR]       = (avr_sensor_data) & (DEV_AVR_LEFT_IR_1);
    cn_node[COLLISION_R] = (avr_sensor_data) & (DEV_AVR_RIGHT_COLLISION);
    cn_node[COLLISION_L] = (avr_sensor_data) & (DEV_AVR_LEFT_COLLISION);
#else
    bool* ir_node = slam_data.ir_node;
    bool* cn_node = slam_data.collision_end_node;
    ir_node[IR_RR] = FALSE;
    ir_node[IR_RF] = FALSE;
    ir_node[IR_FR] = FALSE;
    ir_node[IR_FL] = FALSE;
    ir_node[IR_LF] = FALSE;
    ir_node[IR_LR] = FALSE;
    cn_node[COLLISION_R] = FALSE;
    cn_node[COLLISION_L] = FALSE;
#endif //(FEATURE_SLAM_AVR_SENSOR)

#if (FEATURE_DEMO_TOF_OBSTACLE)
    slam_data.tof_dangerous_zone = APP_SLAM_TOF_DANGER_ZONE_FLAG_NULL;
    // TOF threshold detection
    for (int i = 0; i < lidar_data->data_counter; i ++)
    {
        const uint8_t label = lidar_data->keyframe_label[i];
        if ((DEV_TOF_FIRING_GEOMETRICAL_3 <= label) && (label <= DEV_TOF_FIRING_GEOMETRICAL_13))
        {
            if (lidar_data->dist_mm[i] <= VEHICLE_TOF_OBSTACLE_DIST_MIN)
            {
                // flag the zone
                slam_data.tof_dangerous_zone |= (1<<label);
            }
        }
        else
        {
            if (lidar_data->dist_mm[i] <= VEHICLE_TOF_OBSTACLE_DIST_MIN_CORNER)
            {
                // flag the corner zone
                slam_data.tof_dangerous_zone |= (1<<label);
            }
        }
    }
#endif //(FEATURE_DEMO_TOF_OBSTACLE)
}

static INLINE void app_slam_private_resetGlobalMap(void)
{
    memset(&slam_data.gMap, 0, sizeof(dynamic_map_S));
    slam_data.gMap.map_center_pixel.x = GMAP_DEFAULT_CENTRAL_X_INDEX_PIXEL;
    slam_data.gMap.map_center_pixel.y = GMAP_DEFAULT_CENTRAL_Y_INDEX_PIXEL;
}

/**
 * @brief Translate Global Map & (clear out-of-bound data)
 * 
 * Assume: dx, dy \in [- EDGE, EDGE]
 */
static void app_slam_private_translateGlobalMap(int32_t dx_pixel, int32_t dy_pixel)
{
    map_pixel_data_t* mdata = (slam_data.gMap.data);
    math_cart_coord_int32_S* mc_pixel = &(slam_data.gMap.map_center_pixel);
    int32_t start;
    int32_t end;
    int32_t y_index, x_index;

    // \in [-GMAP_DEFAULT_CENTRAL_X_INDEX_PIXEL, GMAP_DEFAULT_CENTRAL_X_INDEX_PIXEL]
    const int32_t x00 = mc_pixel->x - GMAP_DEFAULT_CENTRAL_X_INDEX_PIXEL;
    const int32_t y00 = mc_pixel->y - GMAP_DEFAULT_CENTRAL_Y_INDEX_PIXEL;
    // reset Vertical Columns
    if (dx_pixel != 0)
    {
        start = x00;
        end = x00;
        if (dx_pixel >= 0)
        {
            end += dx_pixel;
        }
        else
        {
            start += dx_pixel;
        }
        y_index = 0;
        for (int32_t j = 0; j < GMAP_HN_PIXEL; j ++)
        {
            for (int32_t i = start; i < end; i ++)
            {
                x_index = i + MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(i), 0, GMAP_WN_PIXEL)];
                mdata[y_index + x_index] = GRID_CELL_NEUTRAL;
            }
            y_index += GMAP_WN_PIXEL;
        }
    }
    // reset Horizontal Columns
    if (dy_pixel != 0)
    {
        start = y00;
        end = y00;
        if (dy_pixel >= 0)
        {
            end += dy_pixel;
        }
        else
        {
            start += dy_pixel;
        }
        for (int32_t j = start; j < end; j ++)
        {
            y_index = j + MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(j), 0, GMAP_HN_PIXEL)];
            y_index *= GMAP_WN_PIXEL;
            for (int32_t i = 0; i < GMAP_WN_PIXEL; i ++)
            {
                mdata[y_index + i] = GRID_CELL_NEUTRAL;
            }
        }
    }

    // translate dynamic map central indexer
    mc_pixel->x += dx_pixel;
    mc_pixel->y += dy_pixel;
    // map indexer saturation within [0, GMAP_GRID_EDGE_SIZE_PIXEL) = [0, GMAP_WN_PIXEL] range
    mc_pixel->x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(mc_pixel->x), 0, GMAP_WN_PIXEL)];
    mc_pixel->y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(mc_pixel->y), 0, GMAP_HN_PIXEL)];
}

/**
 * @brief Clear current vehicle region on the global map
 * 
 * Assume: map translated
 */
static void app_slam_private_clearVehicleRegion(void)
{
    // [0, W | H )
    const math_cart_coord_int32_S* mc_pixel = &(slam_data.gMap.map_center_pixel);
    const int32_t cx_offsetted = mc_pixel->x - (ROBOT_SIZE_R_PIXEL);
    const int32_t cy_offsetted = mc_pixel->y - (ROBOT_SIZE_R_PIXEL);
    // const int8_t PADDING[ROBOT_SIZE_D_PIXEL + 1U] = {4, 2, 1, 1, 0, 0, 0, 1, 1, 2, 4}; // space skip
    const int8_t PADDING[ROBOT_SIZE_D_PIXEL + 1U] = {5, 3, 2, 2, 1, 1, 1, 2, 2, 3, 5}; // space skip + 1 space padding
    int32_t x,y,dx_pad;

    map_pixel_data_t* mdata = (slam_data.gMap.data);

    for (int32_t j = 0; j <= (ROBOT_SIZE_D_PIXEL); j ++)
    {
        y = cy_offsetted + j;
        y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(y), 0, GMAP_HN_PIXEL)];
        y *= GMAP_WN_PIXEL;
        dx_pad = PADDING[j];

        for (int32_t i = dx_pad; i <= (ROBOT_SIZE_D_PIXEL - dx_pad); i ++)
        {
            x = cx_offsetted + i;
            x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_WN_PIXEL)];
            // set cell visited
            mdata[y + x] = GRID_CELL_VISITED;
        }
    }
}

/**
 * @brief Update map based on IR and Collision status
 * 
 */
static void app_slam_private_updateEdgeRegion(void)
{
    // Cache data pointers
    const bool *                    ir_node = slam_data.ir_node;
    const bool *                    cn_node = slam_data.collision_end_node;
    const edge_sensor_config_S *    s_config = slam_data.sensor_config;
    const int8_t *                  config_node_x_pixel = s_config->edge_node_x_pixel;
    const int8_t *                  config_node_y_pixel = s_config->edge_node_y_pixel;
    const vehicle_edge_node_E *     config_node_ir = s_config->edge_node_ir;
    map_pixel_data_t *              mdata = (slam_data.gMap.data);

    // const data
    const int32_t cx_pixel = slam_data.gMap.map_center_pixel.x;
    const int32_t cy_pixel = slam_data.gMap.map_center_pixel.y;
    const int8_t node_offset = slam_data.gMap.orientation_node;

    // intermediate storage
    map_pixel_data_t old_val, new_val;
    int8_t node;
    int32_t x, y, index;
    
    // Update Collision: 
    new_val = (cn_node[COLLISION_L]) ? GRID_CELL_OCCUPANCY_MAX_PROB : GRID_CELL_VISITED_SENSOR;
    for (int8_t i = VEHICLE_COLLISION_START_NODE; i < VEHICLE_COLLISION_NUM_NODES; i ++)
    {
        node = node_offset + i;
        node = EDGE_NODE_WRAPPING(node);
        x = cx_pixel + config_node_x_pixel[node];
        y = cy_pixel + config_node_y_pixel[node];
        x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_WN_PIXEL)];
        y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(y), 0, GMAP_HN_PIXEL)];
        index = y * GMAP_WN_PIXEL + x;
        // update
        old_val = mdata[index];
        mdata[index] = GRID_CELL_UPDATE(old_val, new_val);
    }

    new_val = (cn_node[COLLISION_R]) ? GRID_CELL_OCCUPANCY_MAX_PROB : GRID_CELL_VISITED_SENSOR;
    for (int8_t i = VEHICLE_COLLISION_START_NODE; i < VEHICLE_COLLISION_NUM_NODES; i ++)
    {
        node = node_offset - i;
        node = EDGE_NODE_WRAPPING(node);
        x = cx_pixel + config_node_x_pixel[node];
        y = cy_pixel + config_node_y_pixel[node];
        x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_WN_PIXEL)];
        y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(y), 0, GMAP_HN_PIXEL)];
        index = y * GMAP_WN_PIXEL + x;
        // update
        old_val = mdata[index];
        mdata[index] = GRID_CELL_UPDATE(old_val, new_val);
    }

    // Update IR:
    for (vehicle_IR_channel_E i = (vehicle_IR_channel_E)0U; i < IR_COUNT; i ++)
    {
        new_val = (ir_node[i]) ? GRID_CELL_EDGE_DEFAULT_PROB : GRID_CELL_VISITED_SENSOR;
        node = node_offset + config_node_ir[i];
        node = EDGE_NODE_WRAPPING(node);
        x = cx_pixel + config_node_x_pixel[node];
        y = cy_pixel + config_node_y_pixel[node];
        x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_WN_PIXEL)];
        y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(y), 0, GMAP_HN_PIXEL)];
        index = y * GMAP_WN_PIXEL + x;
        // update
        old_val = mdata[index];
        mdata[index] = GRID_CELL_UPDATE(old_val, new_val);
    }
}

static void app_slam_private_globalMapUpdate(void)
{
    //// Fetch Data ====== ====== ======
    // TODO: Assume we get (dx, dy) from localization (@Alex)
#if (MOCK)
    float mock_dx_mm = 0.0f; // 1mm / 0.1s => 10mm / s
    float mock_dy_mm = 1.0f;
    float mock_dtheta_rad = 0.0f; // Assume: < pi
#endif // (MOCK)

    //// Update Dynamic Map ===== ======
    // accumulate leftover float bits from previous update
    float dx_mm = mock_dx_mm + slam_data.gMap.map_offset_mm.x;
    float dy_mm = mock_dy_mm + slam_data.gMap.map_offset_mm.y;
    float theta = mock_dtheta_rad + slam_data.gMap.vehicle_orientation_rad;
    // translate translation to pixel space:
    const int32_t dx_pixel = GMAP_MM_TO_UNIT_PIXEL(dx_mm);
    const int32_t dy_pixel = GMAP_MM_TO_UNIT_PIXEL(dy_mm);
    // translate dynamic map
    app_slam_private_translateGlobalMap(dx_pixel, dy_pixel);
    // compute leftover float bits (-10, 10) mm
    dx_mm -= (GMAP_UNIT_PIXEL_TO_MM(dx_pixel));
    dy_mm -= (GMAP_UNIT_PIXEL_TO_MM(dy_pixel));
    // saturate theta to [-pi, pi]
    theta = ANGLE_WRAP_NPI_TO_PI(theta);
    // map robot pose theta => one of the 28 edge nodes:
    const vehicle_edge_node_E orientation_node = EDGE_NODE_MAPPING(theta);
    // store leftover bit
    slam_data.gMap.map_offset_mm.x = dx_mm;
    slam_data.gMap.map_offset_mm.y = dy_mm;
    slam_data.gMap.vehicle_orientation_rad = theta;
    slam_data.gMap.orientation_node = orientation_node;
    // printf("mm: [%f, %f] \n", dx_mm, dy_mm);
    
    //// Update Map Content ===== ======
    // Clear Vehicle Region
    app_slam_private_clearVehicleRegion();
    
    // Map edge IR sensors + collision sensors to map
    app_slam_private_updateEdgeRegion();

    // Map tof obstacles to map
#if (FEATURE_LIDAR)
    // TODO: app_slam_private_updateGmapFromToF();
#endif //(FEATURE_LIDAR)
}

static void app_slam_private_obstacleDetection(void)
{
    // Cache data pointers
    map_pixel_data_t *              mdata = (slam_data.gMap.data);

    // const data
    const int32_t cx_pixel = slam_data.gMap.map_center_pixel.x;
    const int32_t cy_pixel = slam_data.gMap.map_center_pixel.y;

    // intermediate storage
    int32_t x, y, index;
    
    // collision detection
    uint8_t obstacle_count = 0;

    for (int32_t j = - VEHICLE_AVOIDANCE_R_PIXEL; j <= VEHICLE_AVOIDANCE_R_PIXEL; j ++)
    {
        y = cy_pixel + j;
        y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(y), 0, GMAP_HN_PIXEL)];
        index = y * GMAP_WN_PIXEL;

        for (int32_t i = - VEHICLE_AVOIDANCE_R_PIXEL; i <= VEHICLE_AVOIDANCE_R_PIXEL; i ++)
        {
            x = cx_pixel + i;
            x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_WN_PIXEL)];
            
            // check
            if (mdata[index + x] > GRID_CELL_WALKABLE_THRESHOLD_MAX)
            {
                obstacle_count ++;
            }
        }
    }

    // Store solution:
    slam_data.obstacle_count = obstacle_count;
}

static void app_slam_private_pathPlanning(void)
{
    // TODO: if need to generate new path, do partial path planning
}

static void app_slam_private_motionPlanning(void)
{
    // TODO: improve motion planning from path planning
    // TODO: motion feedback:
#if (MOCK)
    bool mock_robot_was_stationary_previously = FALSE;
#endif // (MOCK)
    // Currently: simple obstacle avoidance logic
    const uint8_t obstacle_count = slam_data.obstacle_count;
    int8_t * Vl_motor_mm_s_50ms = slam_data.motion_profile.left_motor_velocity_mm_s_per_50ms;
    int8_t * Vr_motor_mm_s_50ms = slam_data.motion_profile.right_motor_velocity_mm_s_per_50ms;

    if (obstacle_count > OBSTACLE_TOLERANCE)
    {
#if DEBUG_FPRINT_FEATURE_OBSTACLES
        PRINTF("[INFO] Collision Detected (# %d)\n", obstacle_count);
#endif
        // update velocity to rotation
        if (xSemaphoreTake(slam_data.motion_profile_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE) {            
            // rotate:
            Vl_motor_mm_s_50ms[0U] = - VELOCITY_MIN_MM_S;
            Vr_motor_mm_s_50ms[0U] =   VELOCITY_MIN_MM_S;
            Vl_motor_mm_s_50ms[1U] = - VELOCITY_SOFT_MM_S;
            Vr_motor_mm_s_50ms[1U] =   VELOCITY_SOFT_MM_S;
            // in case class update incomplete => keep spinning
            Vl_motor_mm_s_50ms[2U] = - VELOCITY_SOFT_MM_S;
            Vr_motor_mm_s_50ms[2U] =   VELOCITY_SOFT_MM_S;
            Vl_motor_mm_s_50ms[3U] = - VELOCITY_SOFT_MM_S;
            Vr_motor_mm_s_50ms[3U] =   VELOCITY_SOFT_MM_S;
            slam_data.motion_profile.update_stamp_100ms += 1;
            xSemaphoreGive(slam_data.motion_profile_mutex); // release lock
        }
    }
    else
    {
        // update velocity to rotation
        if (xSemaphoreTake(slam_data.motion_profile_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE) {
            if (mock_robot_was_stationary_previously)
            {
                // Soft-start
                Vl_motor_mm_s_50ms[0U] = VELOCITY_MIN_MM_S;
                Vr_motor_mm_s_50ms[0U] = VELOCITY_MIN_MM_S;
                Vl_motor_mm_s_50ms[1U] = VELOCITY_SOFT_MM_S;
                Vr_motor_mm_s_50ms[1U] = VELOCITY_SOFT_MM_S;
            }
            else
            {
                // Forward
                Vl_motor_mm_s_50ms[0U] = VELOCITY_MAX_MM_S;
                Vr_motor_mm_s_50ms[0U] = VELOCITY_MAX_MM_S;
                Vl_motor_mm_s_50ms[1U] = VELOCITY_MAX_MM_S;
                Vr_motor_mm_s_50ms[1U] = VELOCITY_MAX_MM_S;

            }
            // in case class update incomplete, stop motor:
            Vl_motor_mm_s_50ms[2U] = VELOCITY_ZERO_MM_S;
            Vr_motor_mm_s_50ms[2U] = VELOCITY_ZERO_MM_S;
            Vl_motor_mm_s_50ms[3U] = VELOCITY_ZERO_MM_S;
            Vr_motor_mm_s_50ms[3U] = VELOCITY_ZERO_MM_S;
            slam_data.motion_profile.update_stamp_100ms += 1;
            xSemaphoreGive(slam_data.motion_profile_mutex); // release lock
        }
    }
}

#if (DEBUG_FPRINT_FEATURE_MAP)
static void app_slam_private_debugPrintMap(bool central, int32_t count)
{
    // [0, W | H )
    const int32_t cx= slam_data.gMap.map_center_pixel.x - GMAP_DEFAULT_CENTRAL_X_INDEX_PIXEL;
    const int32_t cy= slam_data.gMap.map_center_pixel.y - GMAP_DEFAULT_CENTRAL_Y_INDEX_PIXEL;
    map_pixel_data_t* mdata = (slam_data.gMap.data);
    int32_t x,y;
    if (central)
    {
        PRINTF("MAP-Centered: , %d, \n", count);
        for (int32_t j = 0; j < GMAP_WN_PIXEL; j ++)
        {
            y = cy + j;
            y += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(y), 0, GMAP_HN_PIXEL)];
            y *= GMAP_WN_PIXEL;
            for (int32_t i = 0; i < GMAP_HN_PIXEL; i ++)
            {
                x = cx + i;
                x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_HN_PIXEL)];
                PRINTF("%d,", mdata[y + x]);
            }
            PRINTF("%d\n", 0);
        }
    }
    else // raw gmap
    {
        PRINTF("MAP-Memory: , %d, \n", count);
        for (int32_t j = 0; j < GMAP_WN_PIXEL; j ++)
        {
            y = j;
            y *= GMAP_WN_PIXEL;
            for (int32_t i = 0; i < GMAP_HN_PIXEL; i ++)
            {
                x = i;
                PRINTF("%d,", mdata[y + x]);
            }
            PRINTF("%d\n", 0);
        }
    }
}
#endif

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_slam_init(void)
{
    // reset
    memset(&slam_data, 0x00, sizeof(app_slam_data_S));
    app_slam_private_resetGlobalMap();

    // init
    slam_data.sensor_config = & edge_sensor_config;
    slam_data.motion_profile_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(slam_data.motion_profile_mutex); // release mutex for usage

    // status resport
    PRINTF("[GMAP] Size: (%d x %d)\n", GMAP_WN_PIXEL, GMAP_HN_PIXEL);
}

void app_slam_run100ms(void)
{
#if (FEATURE_SLAM)
    if (slam_data.mapResetRequested)
    {
        app_slam_private_resetGlobalMap();
        slam_data.mapResetRequested = FALSE;
    }
    app_slam_private_localization();
    app_slam_private_localMapUpdate();
    app_slam_private_globalMapUpdate();
    app_slam_private_obstacleDetection();
    app_slam_private_pathPlanning();
    app_slam_private_motionPlanning();
    
# if (DEBUG_FPRINT_FEATURE_MAP)
    static int count = 0;
    count ++;
    if (count % 10 == 0)
    {
        app_slam_private_debugPrintMap(DEBUG_FPRINT_FEATURE_MAP_CENTERED, count);
    }
# endif
#endif //(FEATURE_SLAM)
}

uint8_t app_slam_getMotionVelocity(int8_t * left_motor_mm_s_50ms, int8_t * right_motor_mm_s_50ms, uint8_t frame_stamp)
{
    motion_profile_S * motion_profile = & slam_data.motion_profile;
    int8_t * Vl_motor_mm_s_50ms = slam_data.motion_profile.left_motor_velocity_mm_s_per_50ms;
    int8_t * Vr_motor_mm_s_50ms = slam_data.motion_profile.right_motor_velocity_mm_s_per_50ms;
    if (xSemaphoreTake(slam_data.motion_profile_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE) {            
        if (motion_profile->update_stamp_100ms != frame_stamp)
        {
            frame_stamp = motion_profile->update_stamp_100ms;
            motion_profile->read_index = 0U;
        }
        uint8_t index = motion_profile->read_index;
        * left_motor_mm_s_50ms  = Vl_motor_mm_s_50ms[index];
        * right_motor_mm_s_50ms = Vr_motor_mm_s_50ms[index];
        index ++;
        motion_profile->read_index = (index >= VELOCITY_BUFFER_SIZE)?(VELOCITY_BUFFER_SIZE - 1):index;
        xSemaphoreGive(slam_data.motion_profile_mutex); // release lock
    }
    return frame_stamp;
}

void app_slam_requestToResetMap(void)
{
    slam_data.mapResetRequested = TRUE;
}

uint16_t app_slam_requestToFDangerZone(void)
{
    uint16_t status = APP_SLAM_TOF_DANGER_ZONE_FLAG_NULL;
#if (FEATURE_DEMO_TOF_OBSTACLE)
    status = slam_data.tof_dangerous_zone;
#endif //(FEATURE_DEMO_TOF_OBSTACLE)
    return status;
}
