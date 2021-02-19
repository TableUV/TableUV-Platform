/**
 * @file collision.c
 * @author Jerome Villapando
 * @date 15 Feb 2021
 * @brief Device configure files
 *
 * This document will contains device configure content
 */

#include "collision.h"


/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
typedef struct{
    volatile bool left_col_pressed;
    volatile bool right_col_pressed;
} collision_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void collision_private_gpio_config(void);



///////////////////////////
///////   DATA     ////////
///////////////////////////
collision_data_S col_data;
volatile bool col_pressed = false;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
ISR(PCINT0_vect)
{
    col_pressed = !col_pressed;

    if(PINB & _BV(RIGHT_COLLISION))
    {
        col_data.right_col_pressed = true;
    }
    else if(PINB & _BV(LEFT_COLLISION))
    {
        col_data.right_col_pressed = true;
    }
    else if(PINB & _BV(RIGHT_COLLISION == 0)
    {
        col_data.right_col_pressed = false;
    }
    else if(PINB & _BV(LEFT_COLLISION == 0)
    {
        col_data.right_col_pressed = false;
    }
}

static inline void collision_private_gpio_config(void)
{
    // Enable PCINT8 to PCINT11
    GIMSK |= _BV(PCIE1);
    // Enable PCINT8 and PCINT9 for left and right collision
    PCMSK1 |= _BV(PCINT8);
    PCMSK1 |= _BV(PCINT9);
}

///////////////////////////////////////
///////   PUBLIC FUNCTION     /////////
///////////////////////////////////////
void collision_init(void)
{
    collision_gpio_config();
}

