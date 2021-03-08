/**
 * @file    app_supervisor.c
 * @author  Jianxiang (Jack) Xu
 * @date    15 Feb 2021
 * @brief   App level files
 *
 * This document will contains supervisor content
 */

#include "app_supervisor.h"

// Std. Lib
#include <stdio.h>
#include <string.h>

// TableUV Lib
#include "common.h"
#include "app_slam.h"

// External Library

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    
} app_supervisor_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////

///////////////////////////
///////   DATA     ////////
///////////////////////////
static app_supervisor_data_S supervisor_data;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////


///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_supervisor_init(void)
{
    memset(&supervisor_data, 0x00, sizeof(app_supervisor_data_S));
}

void app_supervisor_run20ms(void)
{
    // TODO: to be implemented
#if (FEATURE_SUPER_USE_PROFILED_MOTIONS | MOCK)
    uint8_t frame=0U;
    int8_t rmotor, lmotor;
    frame = app_slam_getMotionVelocity(& lmotor, & rmotor, frame);
    PRINTF("[MOTOR] R:%3d, L:%3d (f:%3d) \n", lmotor, rmotor, frame);
#endif
}


