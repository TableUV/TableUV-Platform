/**
 * @file uart_attiny.c
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include <stdio.h>
#include "ir_sensor.h"



/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void ir_private_gpio_config(void);



///////////////////////////
///////   DATA     ////////
///////////////////////////
static ir_data_S ir_data;



////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void ir_private_gpio_config(void)
{
    /* this function initialises the ADC 

        ADC Notes

    Prescaler

    ADC Prescaler needs to be set so that the ADC input frequency is between 50 - 200kHz.

    Example prescaler values for various frequencies

    Clock   Available prescaler values
    ---------------------------------------
        1 MHz   8 (125kHz), 16 (62.5kHz)
        4 MHz   32 (125kHz), 64 (62.5kHz)
        8 MHz   64 (125kHz), 128 (62.5kHz)
    16 MHz   128 (125kHz)

    below example set prescaler to 16 for mcu running at 1MHz
    */

   // Set Prescaler to 64 for 125kHz
    ADCSRA = 
            (1 << ADEN)  |     // Enable ADC 
            (1 << ADPS2) |     // set prescaler to 16, bit 2 
            (1 << ADPS1) |     // set prescaler to 16, bit 1 
            (0 << ADPS0);      // set prescaler to 16, bit 0 
            
    ADCSRB = 
            (1 << ADLAR);      // left shift result (for 8-bit values)
        
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void ir_attiny_init(void)
{
    ir_private_gpio_config();
}

void ir_sensor_update(void)
{
    // Front IR sensors
    ADMUX = IR_FRONT_1_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_data.ir_front_1_data = ADCH;

    ADMUX = IR_FRONT_2_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_data.ir_front_2_data = ADCH;

    // Right IR Sensors
    ADMUX = IR_RIGHT_1_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_data.ir_right_1_data = ADCH;

    ADMUX = IR_RIGHT_2_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_data.ir_right_2_data = ADCH;

    // Left IR Sensors
    ADMUX = IR_LEFT_1_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_data.ir_left_1_data = ADCH;

    ADMUX = IR_LEFT_2_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_data.ir_left_2_data = ADCH;    
}

ir_data_S ir_sensor_get(void)
{
    return ir_data;
}

ir_data_S ir_sensor_retrieve(void)
{
    ir_data_S ir_temp_data;

    // Front IR sensors
    ADMUX = IR_FRONT_1_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_temp_data.ir_front_1_data = ADCH;

    ADMUX = IR_FRONT_2_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_temp_data.ir_front_2_data = ADCH;

    // Right IR Sensors
    ADMUX = IR_RIGHT_1_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_temp_data.ir_right_1_data = ADCH;

    ADMUX = IR_RIGHT_2_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_temp_data.ir_right_2_data = ADCH;

    // Left IR Sensors
    ADMUX = IR_LEFT_1_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_temp_data.ir_left_1_data = ADCH;

    ADMUX = IR_LEFT_2_MUX;
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    ir_temp_data.ir_left_2_data = ADCH;

    return ir_temp_data;
}

void ir_test_code(void) 
{
  
    ir_attiny_init();

    ir_data_S ir_sensor_data;

    while(1)
    {
        ir_sensor_data = ir_sensor_retrieve();

        char my_string[7];

        sprintf(my_string, "IR_DATA = %d\n", ir_sensor_data.ir_left_2_data);
        // UART_tx_str(my_string);

        _delay_ms(500/8);
    }
}

