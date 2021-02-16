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

// SDK config 
#include "sdkconfig.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define ESP32_CORE_LOW_LEVEL    (0U)
#define ESP32_CORE_HIGH_LEVEL   (1U)

#define TASK_10MS               (10 / portTICK_PERIOD_MS)   // 100 [Hz]
#define TASK_100MS              (100 / portTICK_PERIOD_MS)  //  10 [Hz]
#define TASK_500MS              (500 / portTICK_PERIOD_MS)  //   2 [Hz]
#define TASK_1000MS             (1000 / portTICK_PERIOD_MS) //   1 [Hz]

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static void esp32_task_init(void);

static void core0_task_run10ms(void * pvParameters);
static void core0_task_run100ms(void * pvParameters);
static void core0_task_run1000ms(void * pvParameters);
static void core1_task_run500ms(void * pvParameters);

///////////////////////////
///////   DATA     ////////
///////////////////////////

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static void core0_task_run10ms(void * pvParameters)
{
    /* Block for 10ms. */
    const TickType_t xDelay = TASK_10MS;

    for( ;; )
    {
        /* Do sth at */
        {
            //  add task
            dev_run10ms();
        }
        vTaskDelay( xDelay );
    }
}
static void core0_task_run100ms(void * pvParameters)
{
    /* Block for 100ms. */
    const TickType_t xDelay = TASK_100MS;

    for( ;; )
    {
        /* Do sth at */
        {
            //  add task
            dev_run100ms();
        }
        vTaskDelay( xDelay );
    }
}
static void core0_task_run1000ms(void * pvParameters)
{
    /* Block for 1000ms. */
    const TickType_t xDelay = TASK_1000MS;

    for( ;; )
    {
        /* Do sth at */
        {
            //  add task
            dev_run1000ms();
        }
        vTaskDelay( xDelay );
    }
}

static void core1_task_run500ms(void * pvParameters)
{
    // 20 Hz Update
    /* Block for 500ms. */
    const TickType_t xDelay = TASK_500MS;

    for( ;; )
    {
        /* Do sth at */
        {
            //  add task (High Level)
        }
        vTaskDelay( xDelay );
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
    vTaskDelay(TASK_500MS);

    xTaskCreatePinnedToCore(
        core0_task_run100ms,    /* Function to implement the task */
        "core0_task_run100ms",  /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        2,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_LOW_LEVEL    /* Core where the task should run */
    );  
    vTaskDelay(TASK_500MS);

    xTaskCreatePinnedToCore(
        core0_task_run1000ms,   /* Function to implement the task */
        "core0_task_run1000ms", /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        3,                      /* Priority of the task */
        NULL,                   /* Task handle. */
        ESP32_CORE_LOW_LEVEL    /* Core where the task should run */
    );  
    vTaskDelay(TASK_500MS);

    //  High Level Core Init.
    xTaskCreatePinnedToCore(
        core1_task_run500ms,    /* Function to implement the task */
        "core1_task_run500ms",  /* Name of the task */
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

    // forever loop
    while (true){};
}