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

// External Lib
#include "ICM_20948.h"

/////////////////////////////////
///////   DEFINITION     ////////
/////////////////////////////////
#define SERIAL_PORT Serial
#define SPI_PORT SPI

typedef struct {
    ICM_20948_SPI myICM;
} dev_imu_data_S;

/////////////////////////////////////////
///////   PRIVATE PROTOTYPE     /////////
/////////////////////////////////////////
static inline void dev_imu_private_gpio_config(void);
static inline void dev_imu_private_setup(void);
static inline void dev_imu_test_start(void);
static inline void dev_imu_printScaledAGMT(ICM_20948_SPI*);
static inline void dev_imu_printFormattedFloat(float, uint8_t, uint8_t);

///////////////////////////
///////   DATA     ////////
///////////////////////////
static dev_imu_data_S imu_data;

////////////////////////////////////////
///////   PRIVATE FUNCTION     /////////
////////////////////////////////////////
static inline void dev_imu_private_gpio_config(void)
{
  SPI_PORT.begin(SCLK_3V3, SDO_3V3, SDI_3V3, CS_3V3);
}

static inline void dev_imu_private_setup(void)
{
    SERIAL_PORT.begin(115200);
    while(!SERIAL_PORT){};
    
    bool initialized = false;
    while(!initialized) {
        imu_data.myICM.begin( CS_3V3, SPI_PORT );

        SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
        SERIAL_PORT.println( imu_data.myICM.statusString() );
        if( imu_data.myICM.status != ICM_20948_Stat_Ok ) {
            SERIAL_PORT.println( "Trying again..." );
            delay(500);
        } else {
            initialized = true;
        }
    }
}

static inline void dev_imu_test_start(void) 
{
    while(1) {
        if(imu_data.myICM.dataReady()) {
            imu_data.myICM.getAGMT();
//          printRawAGMT( myICM.agmt );    
            dev_imu_printScaledAGMT( &imu_data.myICM );   
            delay(30);
        } else {
            SERIAL_PORT.println("Waiting for data");
            delay(500);
        }
    }
}

void dev_imu_printScaledAGMT( ICM_20948_SPI *sensor ) {
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  dev_imu_printFormattedFloat( sensor->accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  dev_imu_printFormattedFloat( sensor->accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  dev_imu_printFormattedFloat( sensor->accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  dev_imu_printFormattedFloat( sensor->gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  dev_imu_printFormattedFloat( sensor->gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  dev_imu_printFormattedFloat( sensor->gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  dev_imu_printFormattedFloat( sensor->magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  dev_imu_printFormattedFloat( sensor->magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  dev_imu_printFormattedFloat( sensor->magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  dev_imu_printFormattedFloat( sensor->temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void dev_imu_printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
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


