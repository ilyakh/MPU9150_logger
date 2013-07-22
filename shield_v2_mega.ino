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

#define MPU_UPDATE_RATE  (1000)
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
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of maI jg correction 

#define MPU_LPF_RATE                    20 // low pass filter rate and can be between 5 and 188Hz
#define SD_SERIAL_BAUDRATE              921600 // baudrate; serial port for logging

#define MPU_ACCEL_FSR                   4 // defines full-scale range ( +/- 2, 4, 8, 16 )
#define MPU_GYRO_FSR                  1000 // gyro full scale range ( +/- 250, 500, 1000, 2000 )

// #define TALK_TO_USB
// #define MPULIB_DEBUG
// #define ANNOUNCE_ACCEL_RANGE
// #define ANNOUNCE_SAMPLE_RATE


#define SWITCH_PIN 7

#define TIME_LIMIT 0 // (in milliseconds) time limit for any recording session

const char SEPARATOR = ',';

#define RECORD_BUFFER_LENGTH 32 // buffer length (how many records to store before saving to card?)
#define RECORD_FIELDS 6


// BUFFERS (for sensor values and time)
signed short record_buffer[ RECORD_BUFFER_LENGTH ][ RECORD_FIELDS ];  // buffer for the accelerometer and gyro data (short integers)
unsigned long time_buffer[ RECORD_BUFFER_LENGTH ];             // buffer for time (has own buffer due to different type: unsigned long)

int record_buffer_counter = 0; // counter for _both_ time and record buffer


enum RECORD_FIELD_INDICES {
  
  RAW_ACCELEROMETER_X,      //
  RAW_ACCELEROMETER_Y,      //
  RAW_ACCELEROMETER_Z,      //
                            //  ORDERING OF ITEMS IN THE CSV LOG FILE
  RAW_GYRO_X,               //
  RAW_GYRO_Y,               //
  RAW_GYRO_Z,               //
                            //
  // RAW_QUATERNION_W,         //
  // RAW_QUATERNION_X,         //
  // RAW_QUATERNION_Y,         //
  // RAW_QUATERNION_Z          //
};

long  quaternion[4]; 
short raw_gyro[3];           // in hardware units
short raw_accelerometer[3];  // in hardware units
// short raw_magnetometer[3];   // in hardware units

unsigned long last_timestamp;
short sensors;
unsigned char more;
unsigned long timestamp;


unsigned long start_time;
boolean active = false;

////
//  IMPORTANT! This parameter configures the device to either accept 
///
boolean SWITCH_ENABLED = true;

int inactive_state_counter = 0;
int active_state_counter = 0;

int INACTIVE_THRESHOLD = 50;
int ACTIVE_THRESHOLD = 25;




void setup() {
  
  Wire.begin();
  TWBR = 12; // change I2C to 400 mHz
  
  #ifdef TALK_TO_USB
    Serial.begin( 115200 );
  #endif
  
  Serial2.begin( SD_SERIAL_BAUDRATE );
  
  MPU.selectDevice( DEVICE_TO_USE );
  MPU.init( MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_ONLY, MAG_UPDATE_RATE, MPU_LPF_RATE );   // start the MPU
  
  mpu_set_accel_fsr( MPU_ACCEL_FSR ); // sets full-scale range (+/- 2, 4, 8, 16)
  mpu_set_gyro_fsr( MPU_GYRO_FSR ); // (+/- 250, 500, 1000, 2000)
  
  // dmp_set_fifo_rate( MPU_UPDATE_RATE );
  mpu_set_sample_rate( MPU_UPDATE_RATE ); // allerede satt i init
  
  if ( SWITCH_ENABLED ) { pinMode( SWITCH_PIN, INPUT ); }
  
}



void loop() {
  
  if ( switchStatus() ) {
    
    inactive_state_counter = 0;
    
    if ( ! active ) {
      active_state_counter++;
      
      if ( active_state_counter >= ACTIVE_THRESHOLD ) {
        start_time = millis();
        active = true;
        active_state_counter = 0;
      } 
    }
    
  } else {
    
    active_state_counter = 0;
    
    if ( active ) {
      inactive_state_counter++;
      
      if ( inactive_state_counter >= INACTIVE_THRESHOLD ) {
        start_time = 0;
        active = false;
        inactive_state_counter = 0;
      }
      
    } 
    
  }
  
  
  if ( active || (! SWITCH_ENABLED) ) {
    
    if ( millis() < TIME_LIMIT || TIME_LIMIT == 0 ) { // TIME_LIMIT limits the recording sessions, and records until power loss if 0
      
      readSensorsDmpFromFifo();
      // readSensorsFromRegisters();
      
      if ( last_timestamp < timestamp ) { // ensures that the a reading is fresh, and not a duplicate
      
      // mpu_get_accel_reg( raw_accelerometer, &timestamp );
      // mpu_get_gyro_reg( raw_gyro, &timestamp );
        
        // record_buffer[record_buffer_counter][TIME] = timestamp;
        time_buffer[record_buffer_counter] = (timestamp - start_time);
        
        record_buffer[record_buffer_counter][RAW_ACCELEROMETER_X] = raw_accelerometer[VEC3_X];
        record_buffer[record_buffer_counter][RAW_ACCELEROMETER_Y] = raw_accelerometer[VEC3_Y];
        record_buffer[record_buffer_counter][RAW_ACCELEROMETER_Z] = raw_accelerometer[VEC3_Z];
        
        record_buffer[record_buffer_counter][RAW_GYRO_X] = raw_gyro[VEC3_X];
        record_buffer[record_buffer_counter][RAW_GYRO_Y] = raw_gyro[VEC3_Y];
        record_buffer[record_buffer_counter][RAW_GYRO_Z] = raw_gyro[VEC3_Z];
        
        // record_buffer[record_buffer_counter][RAW_QUATERNION_W] = quaternion[0];
        // record_buffer[record_buffer_counter][RAW_QUATERNION_X] = quaternion[1];
        // record_buffer[record_buffer_counter][RAW_QUATERNION_Y] = quaternion[2];
        // record_buffer[record_buffer_counter][RAW_QUATERNION_Z] = quaternion[3];
        
        // increase the counter or if full reset counter and and write buffer contents to external storage
        if ( record_buffer_counter < (RECORD_BUFFER_LENGTH -1) ) {
          record_buffer_counter++;
        } else {
            recordFromBuffer(); // write down all buffered values
            record_buffer_counter = 0; // reset buffer counter
        }
        
      } // time limit
        
        last_timestamp = timestamp;
        
        /*
        raw_accelerometer[VEC3_X] = 0;
        raw_accelerometer[VEC3_Y] = 0;
        raw_accelerometer[VEC3_Z] = 0;
        
        raw_gyro[VEC3_X] = 0;
        raw_gyro[VEC3_Y] = 0;
        raw_gyro[VEC3_Z] = 0;
        */
    
      // }
    
    
    } else {
      
      // one minute has elapsed, do nothing
      while( 1 ) {}
      
    } 
  }
}


/**********************************************************/
/***********   RECORD PROCEDURES **************************/
/**********************************************************/


void recordFromBuffer() {
  
  for ( int i = 0; i < RECORD_BUFFER_LENGTH; i++ ) {
    
    Serial2.print( time_buffer[i] ); separate();
    
    Serial2.print( record_buffer[i][RAW_ACCELEROMETER_X] ); separate();
    Serial2.print( record_buffer[i][RAW_ACCELEROMETER_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_ACCELEROMETER_Z] ); separate();
    
    Serial2.print( record_buffer[i][RAW_GYRO_X] ); separate();
    Serial2.print( record_buffer[i][RAW_GYRO_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_GYRO_Z] ); // separate();
    
    /*
    Serial2.print( record_buffer[i][RAW_QUATERNION_W] ); separate();
    Serial2.print( record_buffer[i][RAW_QUATERNION_X] ); separate();
    Serial2.print( record_buffer[i][RAW_QUATERNION_Y] ); separate();
    Serial2.print( record_buffer[i][RAW_QUATERNION_Z] );
    */
    
    endRecord(); // new line
    
  }
  
}

void readSensorsFromRegisters() {
    mpu_get_accel_reg( raw_accelerometer, &timestamp );
    mpu_get_gyro_reg( raw_gyro, &timestamp );
}


void readSensorsDmpFromFifo() {
    dmp_read_fifo( raw_gyro, raw_accelerometer, quaternion, &timestamp, &sensors, &more );
}

void record() {
  
    Serial2.print( millisFromStart() ); separate();
    
    Serial2.print( raw_accelerometer[VEC3_X] ); separate();
    Serial2.print( raw_accelerometer[VEC3_Y] ); separate();
    Serial2.print( raw_accelerometer[VEC3_Z] ); separate();
    
    Serial2.print( raw_gyro[VEC3_X] ); separate();
    Serial2.print( raw_gyro[VEC3_X] ); separate();
    Serial2.print( raw_gyro[VEC3_X] );
    
    endRecord();
}

/**********************************************************/
/***********   DEVICE STATE       *************************/
/**********************************************************/

boolean switchStatus() {
  // short: returns the current status of the switch
  // important:
  // mind the mapping
  //  - HIGH voltage means OFF
  //  - LOW voltage means ON
  // ... to avoid fluctuations during power loss due to vibration.
  
  if ( digitalRead( SWITCH_PIN ) ) {
    return false; 
  } else {
    return true; 
  }
}

/**********************************************************/
/***********   TIMING FUNCTIONS   *************************/
/**********************************************************/

unsigned long millisFromStart() {
  return millis() - start_time;
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
