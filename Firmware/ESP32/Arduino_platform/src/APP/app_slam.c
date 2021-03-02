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
#include "dev_ToF_Lidar.h"
#include "slam_math.h"

// External Library


/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef uint8_t map_t;

/*** (parameterization) ***/
// Robot Characteristics 
#define ROBOT_SIZE_R_MM                     (100U)  // 100 [mm] => boundary would be (100 + 10/2) = 105 [mm]
// Global Map
#define GMAP_SQUARE_EDGE_SIZE_MM            (1000U) // 1   [m]
#define GMAP_UNIT_GRID_STEP_SIZE_MM         (10U)   // 10  [mm]

/*** (Pre-compile const.) ***/
// Robot Characteristics 
#define ROBOT_SIZE_R_PIXEL                  ((ROBOT_SIZE_R_MM)/(GMAP_UNIT_GRID_STEP_SIZE_MM))
// Global Map 
#define GMAP_GRID_EDGE_SIZE_PIXEL           ((GMAP_SQUARE_EDGE_SIZE_MM)/(GMAP_UNIT_GRID_STEP_SIZE_MM))
#define GMAP_WN_PIXEL                       ((GMAP_GRID_EDGE_SIZE_PIXEL) + (1U))
#define GMAP_HN_PIXEL                       ((GMAP_GRID_EDGE_SIZE_PIXEL) + (1U))
#define GMAP_CENTRAL_X_INDEX_PIXEL          ((GMAP_GRID_EDGE_SIZE_PIXEL) / (2U))
#define GMAP_CENTRAL_Y_INDEX_PIXEL          ((GMAP_GRID_EDGE_SIZE_PIXEL) / (2U))
#define GMAP_VISIBILITY_RANGE_MAX           ((GMAP_SQUARE_EDGE_SIZE_MM)  / (2U))

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
    map_t                       data[GMAP_WN_PIXEL * GMAP_HN_PIXEL];
    math_cart_coord_int32_S     C_coord_pixel;  // \in [0, GMAP_VISIBILITY_RANGE_MAX]
    math_cart_coord_float_S     dC_coord_mm;    // \in [0,10] [mm] | to cache float value within a pixel
} map_S;

typedef struct{
    dev_tof_lidar_sensor_data_S lidar_data;

    // global map info.
    map_S                       gMap;
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

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_slam_data_S slam_data;

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

static void app_slam_private_globalMapUpdate(void)
{
    // TODO: intake  app/localization + app/localMap => Grid Map (GMap)
    // Bilinear interpolation from  lmap -> gmap: sampling lmap
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


