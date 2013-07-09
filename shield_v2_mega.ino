#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>


// #define MPU_MAXIMAL

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at 0x69

#define  DEVICE_TO_USE    1


MPU9150Lib MPU; // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (500)
// #define DMP_SAMPLE_RATE     (200)

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
#define SD_SERIAL_BAUDRATE              921600 // baudrate; serial port for logging

#define MPU_ACCEL_FSR                   4 // defines full-scale range (+/- 2, 4, 8, 16)

// #define TALK_TO_USB
// #define MPULIB_DEBUG
// #define ANNOUNCE_ACCEL_RANGE
// #define ANNOUNCE_SAMPLE_RATE

const char SEPARATOR = ',';

#define RECORD_BUFFER_LENGTH 128
#define RECORD_FIELDS 6

int record_buffer[ RECORD_BUFFER_LENGTH ][ RECORD_FIELDS ];
int record_buffer_counter = 0;

enum RECORD_FIELD_INDEX {
  RAW_ACCELEROMETER_X,      //
  RAW_ACCELEROMETER_Y,      //
  RAW_ACCELEROMETER_Z,      //
                            //  ORDERING OF ITEMS IN THE CSV LOG FILE
  RAW_GYRO_X,               //
  RAW_GYRO_Y,               //
  RAW_GYRO_Z,               //
                            //
  RAW_QUATERNION_W,         //
  RAW_QUATERNION_X,         //
  RAW_QUATERNION_Y,         //
  RAW_QUATERNION_Z          //
};

long  quaternion[4]; 
short raw_gyro[3];           // in hardware units
short raw_accelerometer[3];  // in hardware units
// short raw_magnetometer[3];   // in hardware units


void setup() {
  
  Wire.begin();
  
  #ifdef TALK_TO_USB
    Serial.begin( 115200 );
  #endif
  
  Serial2.begin( SD_SERIAL_BAUDRATE );
  
  MPU.selectDevice( DEVICE_TO_USE );
  MPU.init( MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE );   // start the MPU
  
  mpu_set_accel_fsr( MPU_ACCEL_FSR ); // sets full-scale range (+/- 2, 4, 8, 16)
  dmp_set_fifo_rate( MPU_UPDATE_RATE );
  
  
                                                              #ifdef ANNOUNCE_ACCEL_RANGE
                                                                unsigned char accelerometer_range[2];
                                                                mpu_get_accel_fsr( &accelerometer_range );
                                                                Serial.print( "Accelerometer range: " );
                                                                Serial.println( accelerometer_range );
                                                              #endif
  
  Serial2.print( "Offset: " );
  Serial2.println( millis() );
  
}

void loop() {
    
  short sensors;
  unsigned char more;
  unsigned long timestamp;
  
  dmp_read_fifo( raw_gyro, raw_accelerometer, quaternion, &timestamp, &sensors, &more );
  
  if ( true ) { // leave for ritual reasons
  
    record_buffer[record_buffer_counter][RAW_ACCELEROMETER_X] = raw_accelerometer[VEC3_X];
    record_buffer[record_buffer_counter][RAW_ACCELEROMETER_Y] = raw_accelerometer[VEC3_Y];
    record_buffer[record_buffer_counter][RAW_ACCELEROMETER_Z] = raw_accelerometer[VEC3_Z];
    
    record_buffer[record_buffer_counter][RAW_GYRO_X] = raw_gyro[VEC3_X];
    record_buffer[record_buffer_counter][RAW_GYRO_Y] = raw_gyro[VEC3_Y];
    record_buffer[record_buffer_counter][RAW_GYRO_Z] = raw_gyro[VEC3_Z];
    
    record_buffer[record_buffer_counter][RAW_QUATERNION_W] = raw_gyro[VEC3_Z];
    record_buffer[record_buffer_counter][RAW_QUATERNION_X] = raw_gyro[VEC3_Z];
    record_buffer[record_buffer_counter][RAW_QUATERNION_Y] = raw_gyro[VEC3_Z];
    record_buffer[record_buffer_counter][RAW_QUATERNION_Z] = raw_gyro[VEC3_Z];
    
    
    // increase the coutner or if full reset counter and and write buffer contents to external storage
    if ( record_buffer_counter < (RECORD_BUFFER_LENGTH -1) ) {
      record_buffer_counter++;
    } else {
      // write all buffered values
      recordFromBuffer();
      record_buffer_counter = 0;
    }
    
    
  }
  
  // 
  // mpu_read_fifo( m_rawGyro, m_rawAccel, &timestamp, &sensors, &more );
  // increases the offset counter for the buffer  
  
}


/**********************************************************/
/***********   RECORD PROCEDURES **************************/
/**********************************************************/


void recordFromBuffer() {
  
  for ( int i = 0; i < RECORD_BUFFER_LENGTH; i++ ) {
    
    Serial2.print( record_buffer[i][RAW_ACCELEROMETER_X] ); separate();
    Serial2.print( record_buffer[i][RAW_ACCELEROMETER_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_ACCELEROMETER_Z] ); separate();
    
    Serial2.print( record_buffer[i][RAW_GYRO_X] ); separate();
    Serial2.print( record_buffer[i][RAW_GYRO_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_GYRO_Z] ); separate();
    
    Serial2.print( record_buffer[i][RAW_QUATERNION_W] ); separate();
    Serial2.print( record_buffer[i][RAW_QUATERNION_X] ); separate();
    Serial2.print( record_buffer[i][RAW_QUATERNION_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_QUATERNION_Z] );
    
    endRecord(); // new line
    
  }
  
}


/**********************************************************/
/***********   FORMATTING FUNCTIONS   *********************/
/**********************************************************/

void separate() {
   Serial2.print( SEPARATOR );
}

void endRecord() {
  Serial2.println(); 
}
