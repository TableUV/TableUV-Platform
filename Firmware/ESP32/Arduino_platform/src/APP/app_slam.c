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

// External Library

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    dev_tof_lidar_sensor_data_S lidar_data;
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
        //
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


