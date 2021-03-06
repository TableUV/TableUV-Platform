/**
 * @file main.c
 * @author 
 * @date 18 Feb 2021
 * @brief Main File for AVR SENSOR
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>    
#include <util/delay.h>      
#include <stdint.h>  

#include "../include/pinConfig.h"
#include "collision.h"
#include "uart_attiny.h"
#include "ir_sensor.h"

#define BUFFER_SIZE                     (10U)
#define COLLISION_FILTER_THRESHOLD      (0.8F)
#define IR_FILTER_THRESHOLD             (0.2F)
#define SENSOR_READ_FREQ                (200U) // Max 7kHz otherwise change the timer prescaler
#define FILTER_FREQ                     (20U)
#define SENSOR_FILTER_RATIO             (SENSOR_READ_FREQ/FILTER_FREQ)
#define SCHEDULER_TIMER_COMPARE         (39U)//(8000000U/(SENSOR_READ_FREQ * 1024U))

typedef enum {
    LEFT_COLLISION_BIT,
    RIGHT_COLLISION_BIT,
    IR_FRONT_1_BIT,
    IR_FRONT_2_BIT,
    IR_RIGHT_1_BIT,
    IR_RIGHT_2_BIT,
    IR_LEFT_1_BIT,
    IR_LEFT_2_BIT
} uart_byte_bits_E;

uint8_t ir_f1_data_buf[BUFFER_SIZE] = {0};
uint8_t ir_f2_data_buf[BUFFER_SIZE] = {0};
uint8_t ir_l1_data_buf[BUFFER_SIZE] = {0};
uint8_t ir_l2_data_buf[BUFFER_SIZE] = {0};
uint8_t ir_r1_data_buf[BUFFER_SIZE] = {0};
uint8_t ir_r2_data_buf[BUFFER_SIZE] = {0};
uint8_t ir_index = 0;

uint8_t left_collision_count;
uint8_t right_collision_count;

uint8_t uart_byte_send = 0;

volatile bool sensor_read_stage = false;
volatile bool filter_stage = false;
volatile uint8_t filter_counter = 0;

void IR_filter_check(uint8_t* ir_arr, uint8_t bit_num)
{
    uint16_t ir_sum = 0;
    for(uint8_t i = 0; i < BUFFER_SIZE; i++){
        ir_sum += ir_arr[i];
    }
    if ((ir_sum/BUFFER_SIZE) < (IR_FILTER_THRESHOLD * UINT8_MAX) )
    {
        uart_byte_send |= _BV(bit_num);
    }    
}

void timer_scheduler_init(void)
{
    
    TCCR1A = 0;
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for xhz increments
    OCR1A = SCHEDULER_TIMER_COMPARE; //(uint16_t) (8000000/ (SENSOR_READ_FREQ * 1024));//i.e. 39 = (8*10^6) / (200 Hz *1024) - 1 (must be <65536)

    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);    

    //enable interrupts
    sei();
}

// Sensor read ISR
ISR(TIM1_COMPA_vect)
{
    sensor_read_stage = true;

    filter_counter++;
    if(filter_counter >= SENSOR_FILTER_RATIO)
    {
        filter_stage = true;
        filter_counter = 0;
    }
}

int main()
{
    CLKPR = _BV(CLKPCE); 
    CLKPR = 0;

    collision_init();
    uart_attiny_init();
    ir_attiny_init();
    timer_scheduler_init();


    while(1)
    {
        if (sensor_read_stage)
        {
            // Retreival stage
            collision_data_S cur_col_data = collision_status_retrieve();
            left_collision_count += (uint8_t) cur_col_data.left_col_pressed;
            right_collision_count += (uint8_t) cur_col_data.right_col_pressed;

            ir_data_S cur_ir_data = ir_sensor_retrieve();
            ir_f2_data_buf[ir_index] = cur_ir_data.ir_front_1_data;
            ir_f1_data_buf[ir_index] = cur_ir_data.ir_front_2_data;
            ir_l1_data_buf[ir_index] = cur_ir_data.ir_left_1_data;
            ir_l2_data_buf[ir_index] = cur_ir_data.ir_left_2_data;
            ir_r1_data_buf[ir_index] = cur_ir_data.ir_right_1_data;
            ir_r2_data_buf[ir_index] = cur_ir_data.ir_right_2_data;
            ir_index++;
            if(ir_index >= BUFFER_SIZE) ir_index = 0;

            sensor_read_stage = false;
        }

        if(filter_stage)
        {
            // Filter stage
            if (left_collision_count > COLLISION_FILTER_THRESHOLD * BUFFER_SIZE)
            {
                uart_byte_send |= _BV(LEFT_COLLISION_BIT);
                left_collision_count = 0;
            }
            if (right_collision_count > COLLISION_FILTER_THRESHOLD * BUFFER_SIZE)
            {
                uart_byte_send |= _BV(RIGHT_COLLISION_BIT);
                right_collision_count = 0;
            }

            IR_filter_check(ir_f1_data_buf, IR_FRONT_1_BIT);
            IR_filter_check(ir_f2_data_buf, IR_FRONT_2_BIT);
            IR_filter_check(ir_l1_data_buf, IR_LEFT_1_BIT);
            IR_filter_check(ir_l2_data_buf, IR_LEFT_2_BIT);
            IR_filter_check(ir_r1_data_buf, IR_RIGHT_1_BIT);
            IR_filter_check(ir_r2_data_buf, IR_RIGHT_2_BIT);

            UART_tx(uart_byte_send);
            uart_byte_send = 0;

            filter_stage = false;
        }
    }
}