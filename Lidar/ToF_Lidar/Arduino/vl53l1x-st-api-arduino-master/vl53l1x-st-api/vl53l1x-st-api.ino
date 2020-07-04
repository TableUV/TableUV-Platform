/*******************************************************************************
This sketch file is derived from an example program
(Projects\Multi\Examples\VL53L1X\SimpleRangingExamples\Src\main.c) in the
X-CUBE-53L1A1 Long Distance Ranging sensor software expansion for STM32Cube
from ST, available here:
http://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube-expansion-software/stm32-ode-sense-sw/x-cube-53l1a1.html
The rest of the files in this sketch are from the STSW-IMG007 VL53L1X API from
ST, available here:
http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html
********************************************************************************
COPYRIGHT(c) 2017 STMicroelectronics
COPYRIGHT(c) 2018 Pololu Corporation
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <Wire.h>
#include "vl53l1_api.h"

// By default, this example blocks while waiting for sensor data to be ready.
// Comment out this line to poll for data ready in a non-blocking way instead.
#define USE_BLOCKING_LOOP

// Timing budget set through VL53L1_SetMeasurementTimingBudgetMicroSeconds().
#define MEASUREMENT_BUDGET_MS 10

// Interval between measurements, set through
// VL53L1_SetInterMeasurementPeriodMilliSeconds(). According to the API user
// manual (rev 2), "the minimum inter-measurement period must be longer than the
// timing budget + 4 ms." The STM32Cube example from ST uses 500 ms, but we
// reduce this to 55 ms to allow faster readings.
#define INTER_MEASUREMENT_PERIOD_MS 20

// ROI scan mode setting 

#define TotalWidthOfSPADS            16
#define TotalHeightOfSPADS           16
#define WidthOfSPADsPerZone           4

// ROI scan mode setting 

//scan direction 
#define DEFAULT_HORIZONTAL_ZONE   ((uint8_t) 4)
#define DEFAULT_VERTICAL_ZONE     ((uint8_t) 2)
#define HORIZONTAL_SPAD_STEP      (TotalWidthOfSPADS  / DEFAULT_HORIZONTAL_ZONE)
#define VERTICAL_SPAD_STEP        (TotalHeightOfSPADS / DEFAULT_VERTICAL_ZONE )
//scan direction 
//#define ROI_SCAN_DIRECTION_VERTICAL
#define ROI_SCAN_DIRECTION_HORIZONTAL

//scan type
#define ROI_SCAN_TYPE_LINE
//#define ROI_SCAN_TYPE_SWIVERL

VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
VL53L1_UserRoi_t               RoiConfig;


int status;
unsigned long StartTime = micros();
unsigned long EndTime   = 0 ;
uint8_t firstTimeInterrupt = 1;
  
void setup()
{
  uint8_t byteData;
  uint16_t wordData;
  
  pinMode(8, OUTPUT); 
  digitalWrite(8, LOW);
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // This is the default 8-bit slave address (including R/W as the least
  // significant bit) as expected by the API. Note that the Arduino Wire library
  // uses a 7-bit address without the R/W bit instead (0x29 or 0b0101001).
  Dev->I2cDevAddr = 0x52;

  VL53L1_software_reset(Dev); 
  VL53L1_RdByte(Dev, 0x010F, &byteData);
  //Serial.print(F("VL53L1X Model_ID: "));
  //Serial.println(byteData, HEX);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  //Serial.print(F("VL53L1X Module_Type: "));
  //Serial.println(byteData, HEX);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  //Serial.print(F("VL53L1X: "));
  //Serial.println(wordData, HEX);

  //Serial.println(F("Autonomous Ranging Test"));
  digitalWrite(8, HIGH);
  status = VL53L1_WaitDeviceBooted(Dev);
  digitalWrite(8, LOW);
  status = VL53L1_DataInit(Dev);
  digitalWrite(8, HIGH);
  status = VL53L1_StaticInit(Dev);
  digitalWrite(8, LOW);

  digitalWrite(8, HIGH);
  status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_LITE_RANGING);
  digitalWrite(8, LOW);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
  
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, MEASUREMENT_BUDGET_MS * 1000);
  //status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
  digitalWrite(8, HIGH);
  SetUserROIInit(&RoiConfig); // 4.128us
  digitalWrite(8, LOW);
  status = VL53L1_SetUserROI(Dev, &RoiConfig); //11.128us
  digitalWrite(8, HIGH);
  status = VL53L1_StartMeasurement(Dev);   //VL53L1_init_and_start_range()
  digitalWrite(8, LOW);
  if(status)
  {
    Serial.println(F("VL53L1_StartMeasurement failed"));
    while(1);
  }
}

void loop()
{
#ifdef USE_BLOCKING_LOOP

     // blocking wait for data ready
    status = VL53L1_WaitMeasurementDataReady(Dev);
 
    if(firstTimeInterrupt == 0){
        EndTime   = micros();
        printRangingData(&RoiConfig); 
        //Serial.print(F("Data Time: "));
        //Serial.println(EndTime - StartTime);
        digitalWrite(8, HIGH);
        set_ROI_scan_zone(&RoiConfig);
        status = VL53L1_SetUserROI(Dev, &RoiConfig); //11.128us
        digitalWrite(8, LOW);
        status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
        StartTime = EndTime;
     }
 /* (2) If first interrupt, do not get data, clear interrupt and start */
    else{
      status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
      firstTimeInterrupt = 0;
     }

//
//    if(!status)
//    {
//      EndTime   = micros();
//      printRangingData(&RoiConfig); 
//      Serial.print(F("Data Time: "));
//      Serial.println(EndTime - StartTime);
//      
//      //status = VL53L1_StopMeasurement(Dev);
//      ////set_ROI_scan_line_horizontal(&RoiConfig);
//      //set_ROI_scan_zone(&RoiConfig);
//      //status = VL53L1_SetUserROI(Dev, &RoiConfig); 
//      VL53L1_ClearInterruptAndStartMeasurement(Dev);
//      StartTime = micros();
//    }
//    else
//    {
//      Serial.print(F("Error waiting for data ready: "));
//      Serial.println(status);
//    }


#else

  static uint16_t startMs = millis();
  uint8_t isReady;

  // non-blocking check for data ready
  status = VL53L1_GetMeasurementDataReady(Dev, &isReady);

  if(!status)
  {
    if(isReady)
    {
      printRangingData();
      VL53L1_ClearInterruptAndStartMeasurement(Dev);
      startMs = millis();
    }
    else if((uint16_t)(millis() - startMs) > VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
    {
      Serial.print(F("Timeout waiting for data ready."));
      VL53L1_ClearInterruptAndStartMeasurement(Dev);
      startMs = millis();
    }
  }
  else
  {
    Serial.print(F("Error getting data ready: "));
    Serial.println(status);
  }

  // Optional polling delay; should be smaller than INTER_MEASUREMENT_PERIOD_MS,
  // and MUST be smaller than VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS
  delay(10);

#endif
}

void printRangingData(VL53L1_UserRoi_t *pRoiConfig)
{
  static VL53L1_RangingMeasurementData_t RangingData;

  digitalWrite(8, HIGH);
  status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
  digitalWrite(8, LOW);
  if(!status)
  {
    Serial.print(pRoiConfig->TopLeftX);
    Serial.print(F(" "));
    Serial.print(pRoiConfig->TopLeftY);
    Serial.print(F(" "));
    Serial.print(pRoiConfig->BotRightX);
    Serial.print(F(" "));
    Serial.print(pRoiConfig->BotRightY);
    Serial.print(F(" "));
    Serial.print(RangingData.RangeStatus);
    Serial.print(F(" "));
    Serial.print(RangingData.RangeMilliMeter);
    Serial.print(F(" "));
    Serial.print(RangingData.SignalRateRtnMegaCps/65536.0);
    Serial.print(F(" "));
    Serial.print(RangingData.AmbientRateRtnMegaCps/65336.0);
    Serial.print(F(" "));
    Serial.println(RangingData.EffectiveSpadRtnCount/256.0);
  }
}

void set_ROI_scan_line_horizontal(VL53L1_UserRoi_t *pRoiConfig)
{
  
   if ( (pRoiConfig->BotRightX + 1) < 16 ){
      pRoiConfig->BotRightX = pRoiConfig->BotRightX + 1; 
      pRoiConfig->TopLeftX  = pRoiConfig->TopLeftX  + 1;
   }
   else{
     if ( (pRoiConfig->TopLeftY + 1) < 16 ){
        pRoiConfig->BotRightX  = 3; 
        pRoiConfig->TopLeftX   = 0;
        pRoiConfig->TopLeftY   = pRoiConfig->TopLeftY + 1;
        pRoiConfig->BotRightY  = pRoiConfig->BotRightY + 1;
     }
     else{
        pRoiConfig->TopLeftX  = 0;
        pRoiConfig->TopLeftY  = 3;
        pRoiConfig->BotRightX = 3;
        pRoiConfig->BotRightY = 0;
     }
   }
}

void set_ROI_scan_line_vertical(VL53L1_UserRoi_t *pRoiConfig)
{
  
   if ( (pRoiConfig->TopLeftY + 1) < 16 ){
      pRoiConfig->BotRightY = pRoiConfig->BotRightY + 1; 
      pRoiConfig->TopLeftY  = pRoiConfig->TopLeftY   + 1;
   }
   else{
     if ( (pRoiConfig->BotRightX + 1) < 16 ){
        pRoiConfig->BotRightY  = 3; 
        pRoiConfig->TopLeftY   = 0;
        pRoiConfig->TopLeftX   = pRoiConfig->TopLeftX + 1;
        pRoiConfig->BotRightX  = pRoiConfig->BotRightX + 1;
     }
     else{
        pRoiConfig->TopLeftX  = 0;
        pRoiConfig->TopLeftY  = 3;
        pRoiConfig->BotRightX = 3;
        pRoiConfig->BotRightY = 0;
     }
   }
}

void SetUserROIInit(VL53L1_UserRoi_t *pRoiConfig) 
{
      uint8_t start_coordinate_horizontal, start_coordinate_vertical ;
      start_coordinate_horizontal = TotalWidthOfSPADS  / DEFAULT_HORIZONTAL_ZONE - 1;
      start_coordinate_vertical   = TotalHeightOfSPADS / DEFAULT_VERTICAL_ZONE   - 1;

      // check if the SPAD size is under the minimum SPAD size (min 4 x 4)
      if (start_coordinate_horizontal < (WidthOfSPADsPerZone-1)){
           start_coordinate_horizontal = 3; 
      }
      if (start_coordinate_vertical < (WidthOfSPADsPerZone-1)){
           start_coordinate_vertical = 3; 
      }
      
      pRoiConfig->TopLeftX  = 0;
      pRoiConfig->TopLeftY  = start_coordinate_vertical;
      pRoiConfig->BotRightX = start_coordinate_horizontal;
      pRoiConfig->BotRightY = 0;
}

void set_ROI_scan_zone(VL53L1_UserRoi_t *pRoiConfig)
{

   if ( (pRoiConfig->BotRightX + HORIZONTAL_SPAD_STEP) < 16 ){
      pRoiConfig->BotRightX = pRoiConfig->BotRightX + HORIZONTAL_SPAD_STEP; 
      pRoiConfig->TopLeftX  = pRoiConfig->TopLeftX  + HORIZONTAL_SPAD_STEP;
   }
   else{
     if ( (pRoiConfig->TopLeftY + VERTICAL_SPAD_STEP) < 16 ){
        pRoiConfig->BotRightX  = HORIZONTAL_SPAD_STEP - 1; 
        pRoiConfig->TopLeftX   = 0;
        pRoiConfig->TopLeftY   = pRoiConfig->TopLeftY + VERTICAL_SPAD_STEP;
        pRoiConfig->BotRightY  = pRoiConfig->BotRightY + VERTICAL_SPAD_STEP;
     }
     else{
        pRoiConfig->TopLeftX  = 0;
        pRoiConfig->TopLeftY  = VERTICAL_SPAD_STEP - 1;
        pRoiConfig->BotRightX = HORIZONTAL_SPAD_STEP - 1;
        pRoiConfig->BotRightY = 0;
     }
   }
}
