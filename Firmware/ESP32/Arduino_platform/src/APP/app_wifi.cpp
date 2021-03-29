/**
 * @file app_wifi.cpp
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Wifi platform
 * 
 */

#include "app_wifi.h"

#include <WiFi.h>

// TableUV Lib
#include "common.h"

const char* ssid = "TP-LINK_B2C7EE";
const char* password = "EDB2C7EE";

void app_wifi_init(void) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    printf("Connecting to WiFi ..\n%c", ' ');
    while (WiFi.status() != WL_CONNECTED) {
        PRINTF('.\n%c', ' ');
        delay(1000);
    }
    printf("CONNECTED \n%c", ' ');
}

void app_wifi_run100ms(void) {

}

