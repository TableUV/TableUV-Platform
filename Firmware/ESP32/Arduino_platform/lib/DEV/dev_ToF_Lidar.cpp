/**
 * @file dev_ToF_Lidar.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_ToF_Lidar.h"

// TableUV Lib
#include "../IO/io_ping_map.h"
#include "../../include/common.h"

// Arduino Lib
#include <SparkFun_VL53L1X.h>
#include <Wire.h>

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define TOF_TIMING_BUDGET_MS       (15U)

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
typedef struct{
    SFEVL53L1X          tofs[DEV_TOF_LIDAR_COUNT];
    int32_t             budget_ms[DEV_TOF_LIDAR_COUNT];
    const int32_t       address[DEV_TOF_LIDAR_COUNT];
} dev_tof_lidar_data_S;

///////////////////////////
///////   DATA     ////////
///////////////////////////
dev_tof_lidar_data_S lidar_data = {
    {
        SFEVL53L1X(Wire, 21, TOF_INT_1),//TOF_SHUT, TOF_INT_1),
        SFEVL53L1X(Wire, 22, TOF_INT_2),//TOF_SHUT, TOF_INT_2),
        SFEVL53L1X(Wire, 23, TOF_INT_3),//TOF_SHUT, TOF_INT_3)
    },
    {
        0,0,0
    },
    { 
        0x62, 0x64, 0x66 // Arbitrary Address (non-default)
    },
};

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
void dev_ToF_reset_all_sensors(void)
{
    // start ToFs
    for (int sensor_id = DEV_TOF_LIDAR_R; sensor_id < DEV_TOF_LIDAR_COUNT; sensor_id ++)
    {
        SFEVL53L1X * sensor = & (lidar_data.tofs[sensor_id]);
        if (sensor->begin() == true)
        {
            PRINTF("%d Sensor online!\n", sensor_id);
        }
        else
        {
            PRINTF("%d Sensor offline!\n", sensor_id);
        }

        // Take down all sensors
        sensor->sensorOff();
    }

    delay(100);
    // read sensor initial values & set address & activate sensor
    for (int sensor_id = DEV_TOF_LIDAR_R; sensor_id < DEV_TOF_LIDAR_COUNT; sensor_id ++)
    {
        SFEVL53L1X * sensor = & (lidar_data.tofs[sensor_id]);
        sensor->sensorOn();
        int boot = sensor->checkBootState();

        // set initial values
        sensor->setDistanceModeShort();
        sensor->setTimingBudgetInMs(TOF_TIMING_BUDGET_MS);
        // sensor->setROI(,,);
        // lidar_data.budget_ms[sensor_id] = lidar_data.tofs[sensor_id].getTimingBudgetInMs();
        
        // change address
        sensor->setI2CAddress(lidar_data.address[sensor_id]);
        
        // activate sensor
        sensor->init();
        sensor->startRanging();

        PRINTF("%d [#%d] Sensor Boot Code: %d \n", sensor_id, lidar_data.address[sensor_id], boot);
    }
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_ToF_Lidar_init(void)
{
    // start tof I2C
    Wire.begin(TOF_I2C_SDA, TOF_I2C_SCL);

    // reset ToF
}

void dev_ToF_Lidar_update(void)
{
    for (int sensor_id = DEV_TOF_LIDAR_R; sensor_id < DEV_TOF_LIDAR_COUNT; sensor_id ++)
    {
        int dist_mm = lidar_data.tofs[sensor_id].getDistance();
        int add = lidar_data.tofs[sensor_id].getI2CAddress();
        PRINTF("%d [#%d] Sensor: %d [mm]\n", sensor_id, add, dist_mm);
    }
}
