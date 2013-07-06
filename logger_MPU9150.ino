/*

#include "Wire.h"
#include "RTClib.h"

#include "I2Cdev.h"
#include "MPU6050.h"

#include "SD.h"

#define LED_PIN 13



MPU6050 mpu(0x69);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

bool blinkState = false;

const int chipSelect = 10;
const int LOOP_DELAY = 30; // the i2c communication with the clock chip can be disrupted...
// ... by too frequent writing; you should therefore always have a slight delay before...
// ... continuing to the next loop.



File file;        // represents the log file on the SD-card

char path         [32]; // path buffer (stores the path to current file)
char fileName     [8];  // stores filename
char folderName   [8];  // stores name of the folder


RTC_DS1307 RTC;
RTC_Millis RTC_millis;

void setup() {

    Wire.begin();  
    RTC.begin();  
    Serial.begin(57600);

    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
      // RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    // RTC.adjust(DateTime(__DATE__, __TIME__));    

    
    delay( 15 );    
    

    
    
    // initialize device
    Serial.println("Initializing I2C devices...");

    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println( mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed" );

    
    
    


    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    delay( 15 );
    

    
    // SD
    Serial.print( "Initializing SD card... " );
    pinMode( SS, OUTPUT );
    
    // see if the card is present and can be initialized:
    if ( !SD.begin( chipSelect ) ) {
      Serial.println( "card failed, or not present" );
      // stop
      while(1);
      
      // [+] enable red debug LED
    }
    
    Serial.println( "card initialized." );
    
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
    Serial.println( path );    
    
    if (! file ) {
      Serial.println("error opening datalog");
      // wait forever since we can't write data
      while(1) ;
    }

    
}

void loop() {
  
    // read raw accel/gyro measurements from device
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //mpu.getAcceleration(&ax, &ay, &az);
    //mpu.getRotation(&gx, &gy, &gz);
    
    // update clock
    DateTime now = RTC.now();
    
    if ( file ) {
      
      // current timestamp from the real time clock
      file.print( now.unixtime() ); file.print( "," );
      
      // current timestamp from arduino millis()      
      file.print( millis() ); file.print( "," );
      
      
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
      Serial.println( "Error opening file." ); 
    }
    
    delay( LOOP_DELAY );
    
}
*/
