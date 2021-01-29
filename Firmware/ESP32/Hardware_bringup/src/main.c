/*
#####################################################################
#####################################################################
                    PROJECT: TABLE UV 
#####################################################################
#####################################################################

#####################################################################
                    BOARD    : ESP32-WROOM-32U
                    PROGRAM  : Hardware_Bringup Code
                    DEVELOPER: Tsugumi Murata (github: tsuguminn0401)
                    DESCRIPTION: General code to test ESP32 IO and peripherals 
#####################################################################


*/
// Standard libraries 
#include <string.h>
#include <stdio.h>

// SDK config 
#include "sdkconfig.h"

// Non-Volatile-Storage 
#include "nvs.h"
#include "nvs_flash.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp32/ulp.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/dac.h"
#include "driver/i2c.h"

void app_main()
{
    //pin setup function 
    init_setup();

    while(1) {
        /* Blink off (output low) */
	printf("Turning off the LED\n");
        gpio_set_level(STATUS_RED_LED, 0);
        gpio_set_level(STATUS_GREEN_LED, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /* Blink on (output high) */
	printf("Turning on the LED\n");
        gpio_set_level(STATUS_RED_LED, 1);
        gpio_set_level(STATUS_GREEN_LED, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting DAC output tp 50: \n");
        dac_output_voltage(ESP_DAC , 50);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting DAC output tp 100: \n");
        dac_output_voltage(ESP_DAC , 100);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting DAC output tp 150: \n");
        dac_output_voltage(ESP_DAC , 150);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting DAC output tp 200: \n");
        dac_output_voltage(ESP_DAC , 200);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

    
}