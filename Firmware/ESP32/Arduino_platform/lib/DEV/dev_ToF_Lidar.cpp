/**
 * @file dev_ToF_Lidar.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 * 
 * Datasheet: https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * Driver: https://www.st.com/resource/en/user_manual/dm00562924-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf
 * ROI: https://www.st.com/resource/en/application_note/dm00516219-using-the-programmable-region-of-interest-roi-with-the-vl53l1x-stmicroelectronics.pdf
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
#define TOF_TIMING_BUDGET_MS                (15U)
#define TOF_WIDTH_OF_SPADS_PER_ZONE         (5U) // MIN: 4 pix
#define TOF_HEIGHT_OF_SPADS_PER_ZONE        (6U) // MIN: 4 pix
#define TOF_MAX_DIST_MM                     (1300U) // 1.3 [m] in short range mode
#define TOF_INTERMEDIATE_SETTING_DELAY_MS   (20U)
#define MP_MUTEX_BLOCK_TIME_MS              ((1U)/portTICK_PERIOD_MS)
typedef struct{
    SFEVL53L1X          tofs[DEV_TOF_LIDAR_COUNT];
    const uint8_t       address[DEV_TOF_LIDAR_COUNT];
    const uint8_t       firing_sequence[DEV_TOF_FIRING_KEYFRAME_COUNT];
    const uint8_t       firing_sequence_label[DEV_TOF_LIDAR_COUNT][DEV_TOF_FIRING_KEYFRAME_COUNT];
    uint8_t             prev_firingframe[DEV_TOF_LIDAR_COUNT];
    // Protected By: 'mp_mutex'
    SemaphoreHandle_t               mp_mutex;
    dev_tof_lidar_sensor_data_S     mp_data;
} dev_tof_lidar_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
void dev_ToF_reset_all_sensors(void);

///////////////////////////
///////   DATA     ////////
///////////////////////////
static dev_tof_lidar_data_S lidar_data = {
    .tofs = {
        SFEVL53L1X(Wire, 21, TOF_INT_1),//TOF_SHUT, TOF_INT_1),
        SFEVL53L1X(Wire, 22, TOF_INT_2),//TOF_SHUT, TOF_INT_2),
        SFEVL53L1X(Wire, 23, TOF_INT_3),//TOF_SHUT, TOF_INT_3)
    },
    .address = { 
        0x62, 0x64, 0x66 // Arbitrary Address (non-default)
    },
/**Table of Optical Centers**
*
* 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
* 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
* 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
* 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
* 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
* 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
* 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
* 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

* 127,119,111,103, 95, 87, 79, 71,  63, 55, 47, 39, 31, 23, 15, 7
* 126,118,110,102, 94, 86, 78, 70,  62, 54, 46, 38, 30, 22, 14, 6
* 125,117,109,101, 93, 85, 77, 69,  61, 53, 45, 37, 29, 21, 13, 5
* 124,116,108,100, 92, 84, 76, 68,  60, 52, 44, 36, 28, 20, 12, 4
* 123,115,107, 99, 91, 83, 75, 67,  59, 51, 43, 35, 27, 19, 11, 3
* 122,114,106, 98, 90, 82, 74, 66,  58, 50, 42, 34, 26, 18, 10, 2
* 121,113,105, 97, 89, 81, 73, 65,  57, 49, 41, 33, 25, 17, 9, 1
* 120,112,104, 96, 88, 80, 72, 64,  56, 48, 40, 32, 24, 16, 8, 0 Pin 1 
* 
* [GND]
*
* To set the center, set the pad that is to the right and above the exact center of the region you'd like to measure as your opticalCenter*/
    .firing_sequence = {
        199, 151, 239, 175, 215 
        //Firing Order:  Left <-    [2] - [4] - [1] - [5] - [3]    -> Right
    },
    .firing_sequence_label = {
        DEV_TOF_FIRING_GEOMETRICAL_3, DEV_TOF_FIRING_GEOMETRICAL_1, DEV_TOF_FIRING_GEOMETRICAL_5, DEV_TOF_FIRING_GEOMETRICAL_2, DEV_TOF_FIRING_GEOMETRICAL_4,
        DEV_TOF_FIRING_GEOMETRICAL_8, DEV_TOF_FIRING_GEOMETRICAL_6, DEV_TOF_FIRING_GEOMETRICAL_10, DEV_TOF_FIRING_GEOMETRICAL_7, DEV_TOF_FIRING_GEOMETRICAL_9,
        DEV_TOF_FIRING_GEOMETRICAL_13, DEV_TOF_FIRING_GEOMETRICAL_11, DEV_TOF_FIRING_GEOMETRICAL_15, DEV_TOF_FIRING_GEOMETRICAL_12, DEV_TOF_FIRING_GEOMETRICAL_14,
    },
    .prev_firingframe = {
        DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
        DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
        DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
    },
    .mp_mutex = xSemaphoreCreateBinary(),
    .mp_data ={
        .dist_mm = {
            TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
            TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
            TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
            TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
            TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
            TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
        },
        .keyframe_label = {
            DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
            DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
            DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
            DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
            DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
            DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN, DEV_TOF_FIRING_KEYFRAME_UNKNOWN,
        },
        .data_counter = 0U,
    },
};

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
/**
 * @brief This function initialize I2C and GPIO interfaces for all listed Lidars
 * 
 * More specifically, it will power each Lidar one by one, and reassign the register of the Lidar.
 */
void dev_ToF_reset_all_sensors(void)
{
    // buffer
    SFEVL53L1X * sensor;
	// int16_t offset;

    // start ToFs
    for (int sensor_id = DEV_TOF_LIDAR_R; sensor_id < DEV_TOF_LIDAR_COUNT; sensor_id ++)
    {
        sensor = & (lidar_data.tofs[sensor_id]);
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

    delay(TOF_INTERMEDIATE_SETTING_DELAY_MS);

    // read sensor initial values & set address & activate sensor
    for (int sensor_id = DEV_TOF_LIDAR_R; sensor_id < DEV_TOF_LIDAR_COUNT; sensor_id ++)
    {
        sensor = & (lidar_data.tofs[sensor_id]);

        // turn on sensor
        sensor->sensorOn();
        int boot = sensor->checkBootState();
        
        // change address
        sensor->setI2CAddress(lidar_data.address[sensor_id]);
        
        PRINTF("%d [#%d] Sensor Boot Code: %d \n", sensor_id, lidar_data.address[sensor_id], boot);
        
        // activate sensor
        sensor->init();

        // set initial values
        sensor->setDistanceModeShort();
        sensor->setTimingBudgetInMs(TOF_TIMING_BUDGET_MS);
        sensor->setIntermeasurementPeriod(TOF_TIMING_BUDGET_MS);
        sensor->setROI(TOF_WIDTH_OF_SPADS_PER_ZONE, TOF_HEIGHT_OF_SPADS_PER_ZONE, lidar_data.firing_sequence[DEV_TOF_FIRING_KEYFRAME_0]);
        lidar_data.prev_firingframe[sensor_id] = DEV_TOF_FIRING_KEYFRAME_0;
        // offset = sensor->getOffset();
        // sensor->setOffset(offset + 40);
        
        // begin firing
        sensor->startRanging();
        delay(TOF_INTERMEDIATE_SETTING_DELAY_MS);
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
    dev_ToF_reset_all_sensors();
}

void dev_ToF_Lidar_update20ms(void)
{
    /*
     * Idea: Fetch Data and Set new firing ROI for every sensor per 20 ms
     *  - Skip/Drop: if busy, if data mutex busy over 1ms
     */
    uint8_t data_index;
    SFEVL53L1X * sensor;
    VL53L1_Error error;
    uint16_t dist_mm;
    uint8_t firing_frame;
    uint8_t firing_frame_new;
    uint8_t geo_label;
    
    // firing
    for (uint8_t sensor_id = DEV_TOF_LIDAR_R; sensor_id < DEV_TOF_LIDAR_COUNT; sensor_id ++)
    {
        sensor = & (lidar_data.tofs[sensor_id]);
        // if data ready
        if (sensor->checkForDataReady()) 
        {
            // fetch data
            error = sensor->getRangeStatus();
            dist_mm = sensor->getDistance();

            // set new firing pattern
            firing_frame = lidar_data.prev_firingframe[sensor_id];
            firing_frame_new = firing_frame + 1;
            if (firing_frame_new >= DEV_TOF_FIRING_KEYFRAME_COUNT)
            {
                firing_frame_new = DEV_TOF_FIRING_KEYFRAME_0;
            }

            PRINTF("%d Sensor: %3d [mm] F:[%d] Label:(%2d) Status:(%d) \n", sensor_id, dist_mm, firing_frame, lidar_data.firing_sequence_label[sensor_id][firing_frame], error);
            // update new firing pattern
            if (sensor->setCenter(lidar_data.firing_sequence[firing_frame_new]) == VL53L1_ERROR_NONE)
            {
                lidar_data.prev_firingframe[sensor_id] = firing_frame_new;
            }
            sensor->clearInterrupt();

            // store data
            if (error == VL53L1_ERROR_NONE)
            {
                geo_label = lidar_data.firing_sequence_label[sensor_id][firing_frame];
                if (xSemaphoreTake(lidar_data.mp_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE) {
                    // store data to the buffer | Assumption: data shall be consumed before it reach max size
                    data_index = lidar_data.mp_data.data_counter;
                    if (data_index >= DEV_TOF_BUFFER_SIZE)
                    {
                        data_index = 0U; // reset index & override data
                    }
                    lidar_data.mp_data.dist_mm[data_index] = dist_mm;
                    lidar_data.mp_data.keyframe_label[data_index] = geo_label;
                    // safely increment counter
                    lidar_data.mp_data.data_counter = data_index + 1;
                    xSemaphoreGive(lidar_data.mp_mutex); // release lock
                }
            }
            else
            {
                // Do nothing. NOTE: ERROR Handler??
            }
        }
        else
        {
            // Let's check in next 20 [ms]
        }
    }
}

bool dev_ToF_Lidar_dampDataBuffer(dev_tof_lidar_sensor_data_S* buffer)
{
    bool success = FALSE;
    if (xSemaphoreTake(lidar_data.mp_mutex, MP_MUTEX_BLOCK_TIME_MS) == pdTRUE) {
        // memcpy data
        memcpy(buffer, &(lidar_data.mp_data), sizeof(dev_tof_lidar_sensor_data_S));
        // reset counter:
        lidar_data.mp_data.data_counter = 0;
        xSemaphoreGive(lidar_data.mp_mutex); // release lock
        success = TRUE;
    }
    return success;
}
