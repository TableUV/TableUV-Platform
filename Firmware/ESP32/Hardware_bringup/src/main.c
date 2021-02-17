/**
 * @file main.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Main File
 *
 */

// Standard libraries 
#include <string.h>
#include <stdio.h>

// TableUV Lib
#include "dev_config.h"
#include "io_ping_map.h"
#include "APP/app_slam.h"

// SDK config 
#include "sdkconfig.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define ESP32_CORE_LOW_LEVEL        (0U)
#define ESP32_CORE_HIGH_LEVEL       (1U)

#define TASK_SLAM_TASK_TICK         (TASK_HZ_TO_TASK_TICK(  20/*[Hz]*/))
#define TASK_SUPERVISOR_TASK_TICK   (TASK_HZ_TO_TASK_TICK(  50/*[Hz]*/))
#define TASK_100HZ_TASK_TICK        (TASK_HZ_TO_TASK_TICK( 100/*[Hz]*/))
#define TASK_10HZ_TASK_TICK         (TASK_HZ_TO_TASK_TICK(  10/*[Hz]*/))
#define TASK_1HZ_TASK_TICK          (TASK_HZ_TO_TASK_TICK(   1/*[Hz]*/))
#define T_500MS_TASK_TICK           (MS_TO_TASK_TICK(      500/*[ms]*/))

#define MS_TO_TASK_TICK(x)          (TickType_t)((x)/(portTICK_PERIOD_MS))
#define TASK_HZ_TO_TASK_TICK(x)     (MS_TO_TASK_TICK(1000/(x))) // Range: [ < 1 kHz]
/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static void esp32_task_init(void);

static void core0_task_run10ms(void * pvParameters);
static void core0_task_run100ms(void * pvParameters);
static void core0_task_run1000ms(void * pvParameters);
static void core1_task_runSLAM(void * pvParameters);

///////////////////////////
///////   DATA     ////////
///////////////////////////

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static void core0_task_run10ms(void * pvParameters)
{
    for( ;; )
    {
        /* Do sth at */
        {
            //  add task
            dev_run10ms();
        }
        vTaskDelay( TASK_100HZ_TASK_TICK );
    }
}
static void core0_task_run100ms(void * pvParameters)
{
    for( ;; )
    {
        /* Do sth at */
        {
            //  add task
            dev_run100ms();
        }
        vTaskDelay( TASK_10HZ_TASK_TICK );
    }
}
static void core0_task_run1000ms(void * pvParameters)
{
    for( ;; )
    {
        /* Do sth at */
        {
            //  add task
            dev_run1000ms();
        }
        vTaskDelay( TASK_1HZ_TASK_TICK );
    }
}

static void core1_task_runSLAM(void * pvParameters)
{
    for( ;; )
    {
        /* Do sth at */
        {
            //  add task (High Level)
            app_slam_run50ms();
        }
        vTaskDelay( TASK_SLAM_TASK_TICK );
    }
}

static void core1_task_runSupervisor(void * pvParameters)
{
    for( ;; )
    {
        /* Do sth at */
        {
            //  add task (High Level)
        }
        vTaskDelay( TASK_SUPERVISOR_TASK_TICK );
    }
}

static void esp32_task_init()
{
    // Low Level Core Init.
    xTaskCreatePinnedToCore(
        core0_task_run10ms,     /* Function to implement the task */
        "core0_task_run10ms",   /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        1,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_LOW_LEVEL    /* Core where the task should run */
    );  
    vTaskDelay(T_500MS_TASK_TICK);

    xTaskCreatePinnedToCore(
        core0_task_run100ms,    /* Function to implement the task */
        "core0_task_run100ms",  /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        2,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_LOW_LEVEL    /* Core where the task should run */
    );  
    vTaskDelay(T_500MS_TASK_TICK);

    xTaskCreatePinnedToCore(
        core0_task_run1000ms,   /* Function to implement the task */
        "core0_task_run1000ms", /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        3,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_LOW_LEVEL    /* Core where the task should run */
    );  
    vTaskDelay(T_500MS_TASK_TICK);

    //  High Level Core Init.
    xTaskCreatePinnedToCore(
        core1_task_runSLAM,    /* Function to implement the task */
        "core1_task_runSLAM",  /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        1,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_HIGH_LEVEL   /* Core where the task should run */
    );  
    vTaskDelay(T_500MS_TASK_TICK);
    
    xTaskCreatePinnedToCore(
        core1_task_runSupervisor,    /* Function to implement the task */
        "core1_task_runSupervisor",  /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        1,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_HIGH_LEVEL   /* Core where the task should run */
    );  
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void app_main()
{
    // device initialization
    dev_init();

    // esp32 task initialization
    esp32_task_init();

    // app level init
    app_slam_init();

    // forever loop
    while (true){};
}