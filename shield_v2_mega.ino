#include <FlexiTimer2.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at 0x69

#define  DEVICE_TO_USE    1

MPU9150Lib MPU; // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (100)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (20)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

#define MPU_LPF_RATE                    20 // low pass filter rate and can be between 5 and 188Hz
#define SD_SERIAL_BAUDRATE              115200 // badurate; serial port for logging

#define MPU_ACCEL_FSR                   8 // defines full-scale range (+/- 2, 4, 8, 16)

// #define TALK_TO_USB


const int RECORD_RATE = 100; // (in Hz) rate of writing to SD-card
const char SEPARATOR = ',';

volatile int rawAccelX, rawAccelY, rawAccelZ;
volatile int calAccelX, calAccelY, calAccelZ;


void setup() {
  
  Wire.begin();
  
  #ifdef TALK_TO_USB
    Serial.begin( 115200 );
  #endif
  
  MPU.selectDevice( DEVICE_TO_USE );
  MPU.init( MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE );   // start the MPU
  
  mpu_set_accel_fsr( MPU_ACCEL_FSR ); // sets full-scale range (+/- 2, 4, 8, 16)
  
  Serial2.begin( SD_SERIAL_BAUDRATE );

  
  // set up timed events
  
  // FlexiTimer2::set( 1000 / RECORD_RATE, readSensors ); // reads sensor output
  // FlexiTimer2::start();
  
  FlexiTimer2::set( 1000 / RECORD_RATE, record ); // records sensor readings
  FlexiTimer2::start();
}

void loop()
{  
  /*
  // MPU.selectDevice( DEVICE_TO_USE );                         // only needed if device has changed since init but good form anyway
  if ( MPU.read() ) {                                        // get the latest data if ready yet
    
    recordTimeSinceStart();
    separate();
    
    recordRawAccel();
    separate();
    recordCalAccel();
    
    endRecord();
    
  }
  */
  
  MPU.read();
  
  rawAccelX = MPU.m_rawAccel[VEC3_X];
  rawAccelY = MPU.m_rawAccel[VEC3_Y];
  rawAccelZ = MPU.m_rawAccel[VEC3_Z];
  
  calAccelX = MPU.m_calAccel[VEC3_X];
  calAccelY = MPU.m_calAccel[VEC3_Y];
  calAccelZ = MPU.m_calAccel[VEC3_Z];
  
}


/**********************************************************/
/***********   TIMED EVENTS  ******************************/
/**********************************************************/

void record() {
    
    recordTimeSinceStart();
    separate();
    
    recordRawAccel();
    separate();
    recordCalAccel();
    
    endRecord();
  
}

/*

void readSensors() {
   MPU.read();
}

*/



/**********************************************************/
/***********   FORMATTING FUNCTIONS   *********************/
/**********************************************************/


void recordTimeSinceStart() {
  Serial2.print( millis() );
}

void recordRawAccel() {
    Serial2.print( rawAccelX ); separate();
    Serial2.print( rawAccelY ); separate();
    Serial2.print( rawAccelZ ); 
}

void recordCalAccel() {
    Serial2.print( calAccelX ); separate();
    Serial2.print( calAccelY ); separate();
    Serial2.print( calAccelZ ); 
}

void separate() {
   Serial2.print( SEPARATOR );
}

void endRecord() {
  Serial2.println(); 
}
