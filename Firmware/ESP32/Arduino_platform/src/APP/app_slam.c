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

// External Library


/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef int8_t map_pixel_data_t; // [-128, 127]

/*** (parameterization) ***/
// Robot Characteristics 
#define ROBOT_SIZE_D_MM                     (100U)  // 100 [mm] => boundary would be (100 + 10/2 + 10/2) = 110 [mm]
// Global Map
#define GMAP_SQUARE_EDGE_SIZE_MM            (1000U) // 1   [m]
#define GMAP_UNIT_GRID_STEP_SIZE_MM         (10U)   // 10  [mm]

// Grid Occupancy
#define GRID_CELL_NEUTRAL                   (0x0) // <- have to be 0 (Unexplored score)
// Choose Rating between (-1 ~ -20) 
//      minimum: (GRID_CELL_WALKABLE_THRESHOLD_MIN)
//      : regularization term for path planning, 
//        -> -20 => less likely to walk through repetitive path
#define GRID_CELL_VISITED                   (-10)

// Rating in: 0~100 : ToF | MAX for collision switch
#define GRID_CELL_OCCUPANCY_MAX_PROB        (100)
// Rating in: 1~20 + 100 => must not intrude!
#define GRID_CELL_EDGE_MIN_PROB             (101)
#define GRID_CELL_EDGE_MAX_PROB             (120)

// 20 <= val <= 20 : walkable
#define GRID_CELL_WALKABLE_THRESHOLD_MAX    (20)
#define GRID_CELL_WALKABLE_THRESHOLD_MIN    (-20)

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

/*** (Macro Functions) ***/
#define GMAP_MM_TO_UNIT_PIXEL(x_mm)             (int32_t)((x_mm)/(GMAP_UNIT_GRID_STEP_SIZE_MM)) // -ve Ceiling => -1, +ve Flooring => 1
#define GMAP_UNIT_PIXEL_TO_MM(x_pixel)          ((x_pixel) * (GMAP_UNIT_GRID_STEP_SIZE_MM))
#define ARG_RANGE_INCLUSIVE(x_val, min, max)    (uint8_t)(((x_val) >= (min)) + ((x_val) >= (max))) // 0: (-inf, min), 1: [min, max], 2: [max, inf)
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
    map_pixel_data_t                  data[GMAP_WN_PIXEL * GMAP_HN_PIXEL];
    math_cart_coord_int32_S           map_center_pixel;     // \in [0, GMAP_GRID_EDGE_SIZE_PIXEL]
    math_cart_coord_float_S           map_offset_mm;        // \in [-GMAP_UNIT_GRID_STEP_SIZE_MM, GMAP_UNIT_GRID_STEP_SIZE_MM]
} dynamic_map_S;

typedef struct{
    dev_tof_lidar_sensor_data_S lidar_data;

    // global map info.
    dynamic_map_S               gMap;
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

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_slam_data_S slam_data;

// global offset data
static const int32_t MAP_OFFSET[3] = {GMAP_WN_PIXEL, 0, -GMAP_WN_PIXEL};

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
    dev_ToF_Lidar_dampDataBuffer(& (slam_data.lidar_data));

    // TODO: 2. Grab IR + Collision Data

    // 3. Process data
    {
        // TODO: data transformation
    }
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
    int32_t index;

    // reset Vertical Columns
    if (dx_pixel >= 0)
    {
        start = 0;
        end = dx_pixel;
    }
    else
    {
        start = GMAP_WN_PIXEL + dx_pixel; // >= 0
        end = GMAP_WN_PIXEL;
    }
    index = 0;
    for (int32_t j = 0; j < GMAP_HN_PIXEL; j ++)
    {
        for (int32_t i = start; i < end; i ++)
        {
            mdata[index + i] = GRID_CELL_NEUTRAL;
        }
        index += GMAP_WN_PIXEL;
    }
    // reset Horizontal Columns
    if (dy_pixel >= 0)
    {
        start = 0;
        end = dy_pixel;
    }
    else
    {
        start = GMAP_HN_PIXEL + dy_pixel; // >= 0
        end = GMAP_HN_PIXEL;
    }
    index = start * GMAP_WN_PIXEL;
    for (int32_t j = start; j < end; j ++)
    {
        for (int32_t i = 0; i < GMAP_WN_PIXEL; i ++)
        {
            mdata[index + i] = GRID_CELL_NEUTRAL;
        }
        index += GMAP_WN_PIXEL;
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
    const int32_t cx_offsetted = slam_data.gMap.map_center_pixel.x - (ROBOT_SIZE_R_PIXEL);
    const int32_t cy_offsetted = slam_data.gMap.map_center_pixel.y - (ROBOT_SIZE_R_PIXEL);
    const int8_t PADDING[ROBOT_SIZE_D_PIXEL + 1U] = {4, 2, 1, 1, 0, 0, 0, 1, 1, 2, 4}; // space skip
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
            x += MAP_OFFSET[ARG_RANGE_INCLUSIVE((int32_t)(x), 0, GMAP_HN_PIXEL)];

            // set cell visited
            mdata[y + x] = GRID_CELL_VISITED;
        }
    }

}

static void app_slam_private_globalMapUpdate(void)
{
    //// Fetch Data ====== ====== ======
    // TODO: Assume we get (dx, dy) from localization (@Alex)
    float mock_dx_mm = 0.0f;
    float mock_dy_mm = 0.0f;

    //// Update Dynamic Map ===== ======
    // accumulate leftover float bits from previous update
    float dx_mm = mock_dx_mm + slam_data.gMap.map_offset_mm.x;
    float dy_mm = mock_dy_mm + slam_data.gMap.map_offset_mm.y;
    // translate translation to pixel space:
    const int32_t dx_pixel = GMAP_MM_TO_UNIT_PIXEL(dx_mm);
    const int32_t dy_pixel = GMAP_MM_TO_UNIT_PIXEL(dy_mm);
    // compute leftover float bits (-10, 10) mm
    dx_mm -= (GMAP_UNIT_PIXEL_TO_MM(dx_pixel));
    dy_mm -= (GMAP_UNIT_PIXEL_TO_MM(dy_pixel));
    // translate dynamic map
    app_slam_private_translateGlobalMap(dx_pixel, dy_pixel);
    // store leftover bit
    slam_data.gMap.map_offset_mm.x = dx_mm;
    slam_data.gMap.map_offset_mm.y = dy_mm;

    //// Update Map Content ===== ======
    // Clear Vehicle Region
    app_slam_private_clearVehicleRegion();
    // TODO: Map edge IR sensors + collision sensors to map
    {

    }
    // TODO: Map tof obstacles to map
    {

    }
}

static void app_slam_private_obstacleDetection(void)
{
    // TODO: in gMap: 
}

static void app_slam_private_pathPlanning(void)
{
    // TODO: if need to generate new path, do partial path planning
}

static void app_slam_private_motionPlanning(void)
{
    // TODO: plan the motions (velocity arrays for the next 50ms)
}


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_slam_init(void)
{
    memset(&slam_data, 0x00, sizeof(app_slam_data_S));
    app_slam_private_resetGlobalMap();

    PRINTF("[GMAP] Size: (%d x %d)\n", GMAP_WN_PIXEL, GMAP_HN_PIXEL);
}

void app_slam_run100ms(void)
{
    app_slam_private_localization();
    app_slam_private_localMapUpdate();
    app_slam_private_globalMapUpdate();
    app_slam_private_obstacleDetection();
    app_slam_private_pathPlanning();
    app_slam_private_motionPlanning();
}


