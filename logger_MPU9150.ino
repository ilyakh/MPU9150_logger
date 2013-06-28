#include "Wire.h"
#include "RTClib.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#include "SD.h"

#define LED_PIN 13
#define MPU_ADDRESS 0x69

// the address is physically altered to avoid collisions with RTC ...
// ... that is also set to 0x68 and does not allow remapping.

// MPU6050 mpu( MPU_ADDRESS ); // [/] raw-data mode
MPU6050 mpu( MPU_ADDRESS );

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

bool blinkState = false;

const int chipSelect = 10;
const int LOOP_DELAY = 30; // the i2c communication with the clock chip can be disrupted...
// ... by too frequent writing; you should therefore always have a slight delay before...
// ... continuing to the next loop.


////////////////////////////////////////////////
///    DMP    /////////////////////////////////
//////////////////////////////////////////////

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



///////////////////////////////////////////////
///    SD    /////////////////////////////////
/////////////////////////////////////////////

File file;        // represents the log file on the SD-card

char path         [32]; // path buffer (stores the path to current file)
char fileName     [8];  // stores filename
char folderName   [8];  // stores name of the folder


////////////////////////////////////////////////
///    CLOCK    ///////////////////////////////
//////////////////////////////////////////////

RTC_DS1307 RTC;
RTC_Millis RTC_millis;




void setup() {

    Wire.begin();  
    RTC.begin();  
    
    if (! RTC.isrunning()) {
      // Serial.println("RTC is NOT running!");
      // RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    // RTC.adjust(DateTime(__DATE__, __TIME__));    

    
    delay( 15 );    
    
    // Serial.begin(57600);
    
    // initialize device
    // Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    // Serial.println("Testing device connections...");
    // [/] raw-data mode
    // // Serial.println( mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed" );
    


    ////////////////////////////////////////////////
    ///    DMP-MODE    ////////////////////////////
    //////////////////////////////////////////////
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // Serial.println(F("Initializing DMP..."));
    
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        // Serial.println(F(")"));
    }



    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    delay( 15 );
    

    
    // SD
    Serial.print( "Initializing SD card... " );
    pinMode( SS, OUTPUT );
    
    
    // see if the card is present and can be initialized:
    boolean SD_status = SD.begin( chipSelect, 11, 12, 13); // MEGA
    delay( 15 );
    
    if ( !SD_status ) {
      // Serial.println( "card failed, or not present" );
      // stop
      while(1);
      
      // [+] enable red debug LED
    }
    
    // Serial.println( "card initialized." );
    
    // sets the current DATE as the name of FOLDER
    DateTime now = RTC.now();  
    sprintf( folderName,
       "%04d%02d%02d",
       now.year(),
       now.month(),
       now.day()
    );
    
    // sets current TIME as the name of FILE 
    sprintf( fileName,
      "%02d%02d%02d",
      now.hour(),
      now.minute(),
      now.second()
    );   
    
    // creates a directory if it doesn't exist    
    if (! SD.exists( folderName ) ) {
      SD.mkdir( folderName ); 
    }
    delay(15);
    
    // opens up the file we're going to log to
    sprintf( path, "%s/%s.txt", (char*) folderName, (char*) fileName );
    file = SD.open( path, FILE_WRITE );
    
    Serial.print( "Writing path: " );
    // Serial.println( path );    
    
    if (! file ) {
      // Serial.println("error opening datalog");
      // wait forever since we can't write data
      while(1) ;
    }
    
}


/*
void loop() {
  
    // read raw accel/gyro measurements from device
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //mpu.getAcceleration(&ax, &ay, &az);
    //mpu.getRotation(&gx, &gy, &gz);
    
    // update clock
    DateTime now = RTC.now();
    
    if ( file ) {

      // current timestamp from arduino millis()      
      file.print( millis() ); file.print( "," );
      
      // current timestamp from the real time clock
      file.print( now.unixtime() ); file.print( "," );
      
      
      file.print( ax ); file.print( "," ); // acceleration on x-axis
      file.print( ay ); file.print( "," ); // acceleration on y-axis
      file.print( az ); file.print( "," ); // acceleration on z-axis
      file.print( gx ); file.print( "," ); // gyro's x
      file.print( gy ); file.print( "," ); // gyro's y
      file.print( gz ); file.print( "," ); // gyro's z
      file.print( mx ); file.print( "," ); // magnetometer's x
      file.print( my ); file.print( "," ); // magnetometer's y
      file.print( mz ); file.print( "\n"); // magnetometer's z
  
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    
      // end of writing to file
      file.flush(); // makes sure the information is written to the SD-card...
      // ... without closing and re-opening the file
    
    } else {
      // Serial.println( "Error opening file." ); 
    }
    
    delay( LOOP_DELAY );
    
}
*/

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
        
        // end of writing to file
        file.flush(); // makes sure the information is written to the SD-card...
        // ... without closing and re-opening the file        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // update time
    DateTime now = RTC.now();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        
        // current timestamp from arduino millis() 
        file.print( millis() ); file.print( "," );
      
        // current timestamp from the real time clock
        file.print( now.unixtime() ); file.print( "," );
        
        
        
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            file.print(euler[0] * 180/M_PI);       file.print( "," );
            file.print(euler[1] * 180/M_PI);       file.print( "," );
            file.print(euler[2] * 180/M_PI);       file.print( "," );
            
        #endif
        

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            file.print(ypr[0] * 180/M_PI);       file.print( "," );
            file.print(ypr[1] * 180/M_PI);       file.print( "," );
            file.print(ypr[2] * 180/M_PI);       file.print( "," );
            
        #endif
        

        
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            
            file.print(aaReal.x);               file.print( "," );
            file.print(aaReal.y);               file.print( "," );
            file.print(aaReal.z);               file.print( "," );
            
        #endif

        
        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            
            file.print(aaWorld.x);            file.print( "," );
            file.print(aaWorld.y);            file.print( "," );
            file.print(aaWorld.z);            // file.print( "," );
            
        #endif
        
        
        
        
        file.print( "\n" );

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
