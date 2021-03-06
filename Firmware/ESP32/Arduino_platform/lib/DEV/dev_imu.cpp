/**
 * @file dev_imu.cpp
 * @author Dong Jae (Alex) Park
 * @date 18 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_imu.h"

// TableUV Lib
#include "../IO/io_ping_map.h"
#include "../../include/common.h"

// Arduino Lib
#include <ICM_20948.h>

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define ACC_CONST 		(9.80665) // [m/s^2]

typedef struct
{
    SPIClass        spi;
    ICM_20948_SPI   sensor;
} dev_imu_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_imu_private_gpio_config(void);
static inline void dev_imu_private_setup(void);

///////////////////////////
///////   DATA     ////////
///////////////////////////
static dev_imu_data_S imu_data;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_imu_private_gpio_config(void)
{
    imu_data.spi = SPIClass();
    imu_data.spi.begin(SCLK_3V3, MISO_3V3, MOSI_3V3, CS_3V3);
}

static inline void dev_imu_private_setup(void)
{
      bool initialized = false;
      while (!initialized)
      {
        imu_data.sensor.begin(CS_3V3, imu_data.spi);
        if (imu_data.sensor.status != ICM_20948_Stat_Ok)
        {
              delay(100);
        }
        else
        {
              initialized = true;
        }
      }
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_imu_init(void)
{
    memset(&imu_data, 0x00, sizeof(dev_imu_data_S));
    dev_imu_private_gpio_config();
    dev_imu_private_setup();
}

bool dev_imu_get_values(float* data_ptr)
{
    if (!imu_data.sensor.dataReady())
    {
        return false;
    }

    // update imu data
    imu_data.sensor.getAGMT();

    // acc values
    data_ptr[IMU_AXIS_ACC_X] = imu_data.sensor.accX() / 1000 * ACC_CONST; // [m/s^2]
    data_ptr[IMU_AXIS_ACC_Y] = imu_data.sensor.accY() / 1000 * ACC_CONST; // [m/s^2]
    data_ptr[IMU_AXIS_ACC_Z] = imu_data.sensor.accZ() / 1000 * ACC_CONST; // [m/s^2]
    // gyro values
    data_ptr[IMU_AXIS_GYR_X] = imu_data.sensor.gyrX();				      // [DPS]
    data_ptr[IMU_AXIS_GYR_Y] = imu_data.sensor.gyrY();					  // [DPS]
    data_ptr[IMU_AXIS_GYR_Z] = imu_data.sensor.gyrZ();  			      // [DPS]
    // mag values
    data_ptr[IMU_AXIS_MAG_X] = imu_data.sensor.magX();					  // [uT]
    data_ptr[IMU_AXIS_MAG_Y] = imu_data.sensor.magY();					  // [uT]
    data_ptr[IMU_AXIS_MAG_Z] = imu_data.sensor.magZ();					  // [uT]

    return true;
}
