/***************************************************
 BNO055 Calibration Demo 
 By Dave Gundlach
 9/21/2015

 Demonstrates how to calibrate the BNO055 Magnatometer
 
 Hardware:
 Arduino Mega
 Adafruit 9DOF (BNO055)
 Adafruit 2.8" TFT LCD w/ Capacitive Touch and SD card
 Adafruit Power Boost Shield
 Mega Mini Shield (3D printed shield for mounting sensors)

 Reference: For Hardware and associated tutorials and libraries
 visit https://www.adafruit.com/

 Libraries:
  Note, the BNO055 library from Adafruit was modified
  to include calibration functions.
  *** Modified library MUST BE USED for this sketch to work *** 

 History:
 V000 Framework
 V001 Cleanup & Added Comments
 V002 Wider direction arrow, decreased sample rate delay.
 
 ****************************************************/
/* Libraries */

#include <SPI.h>               // I2C for TFT & 9DOF
#include <Adafruit_Sensor.h>   // Sensor Library
#include <Adafruit_BNO055.h>   // 9DOF Library (Library Modified)
//#include <utility/imumaths.h>  // 9DOF

/* Initialise BNO055 9DOF  */
#define BNO055_SAMPLERATE_DELAY_MS (50)     // was 100
Adafruit_BNO055 bno = Adafruit_BNO055(55);



/* Global Variables  */
const float Pi = 3.14159265359;           // Pi constant
float magDir;                             // Direction (Magnetic)
int screen = 1;                           // Screen value
byte calMRL = 0;                          // Mag Cal Values
byte calMRM = 0;
byte calMOXL = 0;
byte calMOXM = 0;
byte calMOYL = 0;
byte calMOYM = 0;
byte calMOZL = 0;
byte calMOZM = 0;

//**********************************
//  Setup
void setup() {
  Serial.begin(115200);                          // Start Serial (Debug)
  // Setup SD Card //
  Serial.print("Initializing SD card...");

  // Start the 9DOF sensor //
  if(!bno.begin()){
    Serial.print("BNO055 not detected");         // Debug
    while(1);
  }
  Serial.println("9DOF (BNO055) started");       // Debug
  setCal();                                      // Set 9DOF Calibration Values
  delay(1000);
  bno.setExtCrystalUse(true);
  
 
  Serial.println("Setup Complete");
  
}
//**********************************
// Loop  //
void loop() {
  sensors_event_t event;                   // Read 9DOF Sensor
  bno.getEvent(&event);


     
      byte mcVal = getCalStat();                                            // Get Calibration Status

  
      if (mcVal == 3){                                                      // Mag cal status indicates complete
        readCal();
  
         Serial.println(calMRL);                                                  // Print value in top left box
   
        Serial.println(calMRM);                                                  // Print value in top Right Box

        Serial.println(calMOXL);
 
        Serial.println(calMOXM);
  
        Serial.println(calMOYL);
   
        Serial.println(calMOYM);
  
        Serial.println(calMOZL);
  
        Serial.println(calMOZM);
   
      }
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
//**********************************


// Writes calibration data to 9DOF sensor//
void setCal(){
   

  
    Serial.println("SD Card Values");                 // Debug
    Serial.println(calMRL);
    Serial.println(calMRM);
    Serial.println(calMOXL);
    Serial.println(calMOXM);
    Serial.println(calMOYL);
    Serial.println(calMOYM);
    Serial.println(calMOZL);
    Serial.println(calMOZM);
   
  
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  delay(25);
  bno.setCalvalMRL(calMRL);                    // Send calibration values to BNO055
  bno.setCalvalMRM(calMRM);
  bno.setCalvalMOXL(calMOXL);
  bno.setCalvalMOXM(calMOXM);
  bno.setCalvalMOYL(calMOYL);
  bno.setCalvalMOYM(calMOYM);
  bno.setCalvalMOZL(calMOZL);
  bno.setCalvalMOZM(calMOZM);
  bno.setMode( bno.OPERATION_MODE_NDOF );    // Put into NDOF Mode
  delay(25);
    
}
//**********************************
// Reads calibration data from BNO055 sensor//
void readCal(){
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  
  calMRL = bno.getCalvalMRL();                 // Read Magnetic calibration values
  calMRM = bno.getCalvalMRM();
  calMOXL = bno.getCalvalMOXL();
  calMOXM = bno.getCalvalMOXM();
  calMOYL = bno.getCalvalMOYL();
  calMOYM = bno.getCalvalMOYM();
  calMOZL = bno.getCalvalMOZL();
  calMOZM = bno.getCalvalMOZM();

  bno.setMode( bno.OPERATION_MODE_NDOF );    // Put into NDOF Mode
  delay(25);

}
//**********************************

//************************************
// Calibration Status
byte getCalStat(){
byte cal = bno.getCalib();
  byte calSys = (0xC0 & cal) >> 6;                // Sys Status (0-3 value)
  byte calGyro = (0x30 & cal) >> 4;
  byte calAccel = (0x0C & cal) >> 2;
  byte calMag = (0x03 & cal) >> 0;                // Mag Status (0-3 value)
  
  //Serial.println(cal, BIN);                     // debug
  Serial.print("System calibration status "); Serial.println(calSys);
  Serial.print("Gyro   calibration status "); Serial.println(calGyro);
  Serial.print("Accel  calibration status "); Serial.println(calAccel);
  Serial.print("Mag    calibration status "); Serial.println(calMag);
  
  delay(1000);
  return calMag;
}



  
