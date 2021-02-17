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

// External Library

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    
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
    // TODO
}

static void app_slam_private_localMapUpdate(void)
{
    // TODO
}

static void app_slam_private_globalMapUpdate(void)
{
    // TODO
}

static void app_slam_private_obstacleDetection(void)
{
    // TODO
}

static void app_slam_private_pathPlanning(void)
{
    // TODO
}

static void app_slam_private_motionPlanning(void)
{
    // TODO
}


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_slam_init(void)
{
    memset(&slam_data, 0x00, sizeof(app_slam_data_S));
}

void app_slam_run50ms(void)
{
    app_slam_private_localization();
    app_slam_private_localMapUpdate();
    app_slam_private_globalMapUpdate();
    app_slam_private_obstacleDetection();
    app_slam_private_pathPlanning();
    app_slam_private_motionPlanning();
}


