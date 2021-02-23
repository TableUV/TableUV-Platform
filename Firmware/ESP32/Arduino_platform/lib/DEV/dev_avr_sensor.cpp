/**
 * @file dev_avr_sensor.c
 * @author Jianxiang (Jack) Xu
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "dev_avr_sensor.h"

#include "io_ping_map.h"
#include "HardwareSerial.h"

#include "../../include/common.h"


/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    bool newData;
    uint8_t sensor_rx_data;
} dev_tof_lidar_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_avr_sensor_private_gpio_config(void);

///////////////////////////
///////   DATA     ////////
///////////////////////////
dev_tof_lidar_data_S sensor_avr_data = {
    false,
    0
};



////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_avr_sensor_private_gpio_config(void)
{
    Serial2.begin(SENSOR_AVR_BAUD, SERIAL_8N1, SENSOR_AVR_UART_RX, SENSOR_AVR_UART_TX);
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void dev_avr_sensor_init(void)
{
    dev_avr_sensor_private_gpio_config();
}

void dev_avr_sensor_uart_update(void)
{
    while (Serial2.available() > 0) 
    {
        sensor_avr_data.sensor_rx_data = Serial2.read();
        sensor_avr_data.newData = true;
    }
}

uint8_t dev_avr_sensor_uart_get(void)
{
    if (sensor_avr_data.newData == true) 
    {
        return sensor_avr_data.sensor_rx_data;
        sensor_avr_data.newData = false;
    }
    return 0;
}

uint8_t dev_avr_sensor_uart_read(void)
{
    dev_avr_sensor_uart_update();
    return dev_avr_sensor_uart_get();
}

// void dev_avr_test_code(void)
// {
//     // printf("Avr Sensor Readings: %c%c%c%c%c%c%c%c\n", BYTE_TO_BINARY(dev_avr_sensor_uart_read()));
//     // delay(500);    
// }
