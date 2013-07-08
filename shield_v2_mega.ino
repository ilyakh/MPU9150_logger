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

#define MPU_UPDATE_RATE  (200)

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

#define MPU_LPF_RATE                    40 // low pass filter rate and can be between 5 and 188Hz
#define SD_SERIAL_BAUDRATE              460800 // badurate; serial port for logging

#define MPU_ACCEL_FSR                   4 // defines full-scale range (+/- 2, 4, 8, 16)

#define TALK_TO_USB
// #define MPULIB_DEBUG
// #define ANNOUNCE_ACCEL_RANGE
// #define ANNOUNCE_SAMPLE_RATE


const char SEPARATOR = ',';


#define RECORD_BUFFER_LENGTH 128
#define RECORD_FIELDS 6


int record_buffer[ RECORD_BUFFER_LENGTH ][ RECORD_FIELDS ];
int record_buffer_counter = 0;

enum RECORD_FIELD_INDEX {
  RAW_ACCEL_X,
  RAW_ACCEL_Y,
  RAW_ACCEL_Z,
  CAL_ACCEL_X,
  CAL_ACCEL_Y,
  CAL_ACCEL_Z
};



volatile int rawAccelX, rawAccelY, rawAccelZ;
volatile int calAccelX, calAccelY, calAccelZ;

long previousTime;

void setup() {
  
  Wire.begin();
  
  #ifdef TALK_TO_USB
    Serial.begin( 115200 );
  #endif
  
  MPU.selectDevice( DEVICE_TO_USE );
  MPU.init( MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE );   // start the MPU
  mpu_set_accel_fsr( MPU_ACCEL_FSR ); // sets full-scale range (+/- 2, 4, 8, 16)
  
  delay( 50 );
  
  #ifdef ANNOUNCE_ACCEL_RANGE
    unsigned char accelerometer_range[2];
    mpu_get_accel_fsr( &accelerometer_range );
    Serial.print( "Accelerometer range: " );
    Serial.println( accelerometer_range );
  #endif
  
  #ifdef ANNOUNCE_SAMPLE_RATE
    unsigned short int sample_rate;
    mpu_get_sample_rate( &sample_rate );    
    Serial.print( "Sample rate: " );
    Serial.println( result );
  #endif
  
  Serial2.begin( SD_SERIAL_BAUDRATE );
  
}

void loop() {
  
  if( MPU.read() ) {
    
    record_buffer[record_buffer_counter][RAW_ACCEL_X] = MPU.m_rawAccel[VEC3_X];
    record_buffer[record_buffer_counter][RAW_ACCEL_Y] = MPU.m_rawAccel[VEC3_Y];
    record_buffer[record_buffer_counter][RAW_ACCEL_Z] = MPU.m_rawAccel[VEC3_Z];
    
    record_buffer[record_buffer_counter][CAL_ACCEL_X] = MPU.m_calAccel[VEC3_X];
    record_buffer[record_buffer_counter][CAL_ACCEL_Y] = MPU.m_calAccel[VEC3_Y];
    record_buffer[record_buffer_counter][CAL_ACCEL_Z] = MPU.m_calAccel[VEC3_Z];
    
    // finally, increases buffer counter
    if ( record_buffer_counter < RECORD_BUFFER_LENGTH ) {
      record_buffer_counter++;
    } else {
      // write all buffered values
      recordFromBuffer();
      record_buffer_counter = 0;
    }
    
  } 
  
}


/**********************************************************/
/***********   TIMED EVENTS  ******************************/
/**********************************************************/

void record() {
    
    recordTimeSinceStart(); separate();
    
    recordRawAccel(); separate();
    recordCalAccel();
    
    endRecord();

}

void recordFromBuffer() {
    
  for ( int i = 0; i < RECORD_BUFFER_LENGTH; i++ ) {
    Serial2.print( record_buffer[i][RAW_ACCEL_X] ); separate();
    Serial2.print( record_buffer[i][RAW_ACCEL_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_ACCEL_Z] ); separate();
    Serial2.print( record_buffer[i][CAL_ACCEL_X] ); separate();
    Serial2.print( record_buffer[i][CAL_ACCEL_Y] ); separate();
    Serial2.print( record_buffer[i][CAL_ACCEL_Z] );  
    
    endRecord();
  }
  

}


/**********************************************************/
/***********   FORMATTING FUNCTIONS   *********************/
/**********************************************************/


// [-] deprecated
void recordTimeSinceStart() {
  Serial2.print( millis() );
}

// [-] deprecated
void recordRawAccel() {
    Serial2.print( rawAccelX ); separate();
    Serial2.print( rawAccelY ); separate();
    Serial2.print( rawAccelZ ); 
}

// [-] deprecated
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
