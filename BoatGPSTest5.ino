#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/dtostrf.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/dtostrf.h>
#include <time.h>      // struct tm, time(&t) to secs
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

/*
   This sketch will use the GPS to provide location (latitude,Logngitude), date and time
   current course and speed.
   It will calculate the course to the selected waypoints.
   This data will be passed through the serial network to the other units for use
   It will also send pwm values to the motor ESCs to drive the motors

   The neopixel will be used to show the status of the board
      brown - started program
      red - finished setup
      blue - waiting for GPS data rgb
      green - successful send of data to TX board
      purple - Port
      yellow - Starboard
*/
// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define SLAVE_ADDRESS 0x60

// define the pin for the Neopixel
#define PIN            8
// How many NeoPixels are attached to the Arduino?, set up Neopixels
#define NUMPIXELS      1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

static const uint32_t GPSBaud = 9600;
String Str_Returned;
double courseToNextWayPoint;
int actualCourse;

int OverrideButton = 0;
int PortMotor = 1500;
int StarboardMotor = 1500;

// define the values for the bno055 calibration
byte calMRL = 99;                          // Mag Cal Values
byte calMRM = 4;
byte calMOXL = 205;
byte calMOXM = 255;
byte calMOYL = 180;
byte calMOYM = 0;
byte calMOZL = 121;
byte calMOZM = 0;

// define the buffer to hold the override instructions from the radio
char buf[10];

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

// The TinyGPS++ object
TinyGPSPlus gps;
int GpsIsReady = 0;

// This  will output on D10, D11, D12,D13 PWM signals between 1500us and 1800us wavelength
// set up initial values for the PWM signals
long PWM_1730 = 1730L;   //half speed
long PWM_1820 = 1820L;   //full speed
long PWM_1500 = 1500L;   // motor stopped


// define the array and values for the waypoints for Narrabeen Lakes
float ArrayLat[] = { -33.716729, -33.717182,  -33.716737,  -33.716532,  -33.716802};
float ArrayLong[] = {151.270391, 151.270742, 151.270795, 151.270319, 151.269956};
int WayPointNo = 0;


unsigned long distancemToNextWayPoint;


int16_t packetnum = 0;  // packet counter, we increment per xmission

// Record to hold location information to be passed to the Master controller and then onto the Motor
// control and comms CPUs

char radiopacket[BUFSIZ + 1];

typedef struct {
  char Recno1 = '1';
  char Date[10];
  char Time[8];
  float Latitude;
  float Longitude;
  int ActualCourse;
  int CalcCourse;
  float Speed;
  int Distance;
}  RECORD1;

RECORD1 rec1;

void init_record1(RECORD1 *rec1);



void setup()
{
  pixels.begin(); // This initializes the NeoPixel library.
  Neopixel(25, 13, 0); //brown
  Wire.begin();
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("Start");

  //  start the GPS
  Serial1.begin(9600);

  Serial.println("gps started");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BN0055 started");

  bno.setExtCrystalUse(true);

  // wait until magnetometer is calibrated to at least 2
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  Serial.println("Calibrate the Compass");
  setCal();
 
//  wait until the gps is sending a valid location
   while (!Serial1.available()) {Serial.println("waiting for Serial1");smartDelay(1000); }
  
    // Read from the GPS device and encode it.
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }
  // If we got a fix the location will be updated.
  while (!gps.location.isValid()) {
    Serial.println("waiting for Vaid Location");
    smartDelay(1000);
  }
   Serial.println("GPS is receiving satellite data");
   Serial.print("latitude = "); Serial.println(gps.location.lat());
   
  //  Setup the PWM signals
  setupPWMSignals();

  Neopixel(45, 0, 0);  //red

}

byte x = 0;



void loop()
{


  // Get a new compass reading
  sensors_event_t event;
  bno.getEvent(&event);

  // Actual course is the compass + - minus the magnetic declination, dependng on where in the world you are
  // For Sydney, the magnetic declination is + 12.6 degrees
  actualCourse    = event.orientation.x + 12.6;;
  Serial.print(" compass = "); Serial.println(event.orientation.x);
  if (actualCourse > 360) {
    actualCourse = actualCourse - 360;
  }
  Serial.print(" direction = "); Serial.println(actualCourse);
  distancemToNextWayPoint =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      ArrayLat[WayPointNo],
      ArrayLong[WayPointNo]) ;
 Serial.println(" distance = "); Serial.println(distancemToNextWayPoint);
 //Serial.println(" latitude = "); Serial.println(gps.location.lat());
  // if the distance to the next waypoint is less than 10 meters, move to next waypoint
  if (distancemToNextWayPoint < 10) ++WayPointNo;

  // if you have reached your last waypoint, stop the motors
  if (WayPointNo > 5) {
    setMotorSpeed (PWM_1500, PWM_1500);
    Serial.println("STOP  You have reached your destination");
  }

  courseToNextWayPoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      ArrayLat[WayPointNo],
      ArrayLong[WayPointNo]);

  // calculate the difference and sign (+ or -) between the current course and the calculated course
  int Diff =  ((int) courseToNextWayPoint - (int) event.orientation.x + 540) % 360 - 180;

  // using the difference to point port or starboard, or dead ahead if less than 4 degrees
  // to turn in one direction, increase power to the opposite motor
  // don't do this if the Overiide switch is on
  if (OverrideButton != 1) {
    if (Diff < 4 && Diff > -4) {
      Serial.println("straight ahead");
      setMotorSpeed (PWM_1730, PWM_1730);
    }
    else if (Diff > 4) {
      Serial.println("turn Starboard");  //yellow
      setMotorSpeed (PWM_1730, PWM_1820);
      Neopixel(64, 64, 0);
    }
    else {
      Serial.println("turn Port");  //purple
      setMotorSpeed (PWM_1820, PWM_1730);
      Neopixel(64, 0, 64);
    }
  }
  smartDelay(1000);  //delay for 1 second

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  // pack up the record1 and send
  input_record1(&rec1);
  //Serial.print("Sent: ");
  //Serial.print(radiopacket);
  //Serial.print("\n");
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(radiopacket);
  Wire.endTransmission();
  Neopixel(0, 64, 0); //green
  delay(500);


  Serial.println("Requesting Data");
  Wire.requestFrom(SLAVE_ADDRESS, 9);

  int bytes = Wire.available();
  Serial.print("Slave sent ");
  Serial.print(bytes);
  Serial.print(" of information\n");
  //char x[bytes+1];
  for (int i = 0; i < bytes; i++)
  {
    buf[i] = Wire.read();
  }
  buf[bytes] = '\0';
  Serial.print("Reply from TX "); Serial.println(buf);
  UnpackReply();
  delay(2000);
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while (millis() - start < ms);
  Serial.println("in Smart Delay ");
//  Serial.println(" latitude in Smart delay = "); Serial.println(gps.location.lat());
}


void setupPWMSignals()
{
  // Pins D11(PA20), D13(PA21)are for the Port motor (TCC2), pins D10(PA18), D12(PA19)are for the Starboardmotor (TCC0)
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D11(PA20), D13(PA21). D10(PA18), D12(PA19)
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;

  // Connect the TCC0 timer to digital output D13 = PA17 = ODD - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC2 (and TC3)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC2_WAVE |= TCC_WAVE_POL(0xF) |            // Reverse the output polarity on all TCC2 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;       // Setup dual slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |            // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;       // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC2_PER = 60000; // Set the frequency of the PWM on TCC2 to 100Hz
  while (TCC2->SYNCBUSY.bit.PER);                // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = 60000; // Set the frequency of the PWM on TCC0 to 100Hz
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization

  // Set the PWM signal to output 1500us and 1500us
  REG_TCC2_CC0 = 9000;                              // TCC2 CC0 - on D11
  REG_TCC2_CC1 = 9000;                             // TCC2 CC1 - on D13
  while (TCC2->SYNCBUSY.bit.CC1);                   // Wait for synchronization

  // Set the PWM signal to output 1500us and 1500us
  REG_TCC0_CC2 = 9000;                            // TCC0 CC0 - on D10
  REG_TCC0_CC3 = 9000;                           // TCC0 CC1 - on D12
  while (TCC0->SYNCBUSY.bit.CC3);                 // Wait for synchronization

  //  TCC2 timer tick and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV4 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  //  TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV4 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  // call delay to settle the signal at 1500us
  delay(5000);
}


void setMotorSpeed (int Starboard, int Port)
// this routine changes the PWM for the two ESCs
// TCC2 = Port, TCC0 = Starboard
// if the override command is sent , then set the PWM to the sent values
{
  if (OverrideButton == 1 ) {
    REG_TCC2_CC0 = PortMotor * 6;                    // TCC2 CC0 - on D11
    REG_TCC2_CC1 = PortMotor * 6;                    // TCC2 CC1 - on D13
    while (TCC2->SYNCBUSY.bit.CC1);             // Wait for synchronization


    REG_TCC0_CC2 = StarboardMotor * 6;               // TCC0 CC0 - on D10
    REG_TCC0_CC3 = StarboardMotor * 6;               // TCC0 CC1 - on D12
    while (TCC0->SYNCBUSY.bit.CC3);            // Wait for synchronization}
  }
  else {

    REG_TCC2_CC0 = Port * 6;                    // TCC2 CC0 - on D11
    REG_TCC2_CC1 = Port * 6;                    // TCC2 CC1 - on D13
    while (TCC2->SYNCBUSY.bit.CC1);             // Wait for synchronization


    REG_TCC0_CC2 = Starboard * 6;               // TCC0 CC0 - on D10
    REG_TCC0_CC3 = Starboard * 6;               // TCC0 CC1 - on D12
    while (TCC0->SYNCBUSY.bit.CC3);            // Wait for synchronization}
  }
  //Serial.print("Starboard = "); Serial.println(Starboard);
  //Serial.print("Port = "); Serial.println(Port);
}


void input_record1(RECORD1 *rec1)

{
  //   put the values into the record
  rec1->Latitude  = gps.location.lat();
  rec1->Longitude = gps.location.lng();
  rec1->ActualCourse    = actualCourse;
  rec1->CalcCourse    = courseToNextWayPoint;
  rec1->Speed     = gps.speed.kmph();
  rec1->Distance = distancemToNextWayPoint;
  sprintf(rec1->Time, "%02d%02d%02d ", gps.time.hour(), gps.time.minute(), gps.time.second());
  Serial.print ("lat = "); Serial.println(rec1->Latitude);
    Serial.print ("long = "); Serial.println(rec1->Longitude);
  sprintf(rec1->Date, "%02d%02d%04d ", gps.date.day(), gps.date.month(), gps.date.year());



  // pack up the fields to be sent to BoatRadio program

  char tmp1[20];
  char tmp2[20];
  char tmp3[20];

  dtostrf(rec1->Latitude, 9, 5, tmp1);
  //  Serial.print("lat record"); Serial.println(rec1->Latitude,6);
  //    Serial.print("lat gps"); Serial.println(gps.location.lat(),6);
  //  Serial.print("lat dtostrfs"); Serial.println(tmp1);
  //    Serial.print("long gps"); Serial.println(gps.location.lng());
  dtostrf(rec1->Longitude, 9, 5, tmp2);
  Serial.print("long"); Serial.println(tmp2);
  dtostrf(rec1->Speed, 3, 1, tmp3);

  sprintf(radiopacket, "%c%-4.4s%-2.2s%-6.6s%-9.9s%-9.9s%3d%3d%3s%5d",
          rec1->Recno1,
          rec1->Date,
          rec1->Date + 6,
          rec1->Time,
          tmp1,
          tmp2,
          rec1->ActualCourse,
          rec1->CalcCourse,
          tmp3,
          rec1->Distance );

  int nlen = strlen(radiopacket);



  ///Serial.print("packet length= ");  Serial.println(nlen);

}

void UnpackReply() {
  char temp[10];
  sprintf(temp, "%-1.1s", buf); OverrideButton = atof(temp);
  sprintf(temp, "%-4.4s", buf + 1); PortMotor = atof(temp);
  sprintf(temp, "%-4.4s", buf + 5); StarboardMotor = atof(temp);

  //Serial.print("OverrideButton = "); Serial.println(OverrideButton);
  //Serial.print("PortMotor = "); Serial.println(PortMotor);
  //Serial.print("StarboardMotor = "); Serial.println(StarboardMotor);

}

void Neopixel(int r, int g, int b) {
  Serial.print(" r="); Serial.print(r); Serial.print(" g="); Serial.print(g);  Serial.print(" b="); Serial.println(b);
  // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(0, pixels.Color(r, g, b));

  pixels.show(); // This sends the updated pixel color to the hardware.

}

// Writes calibration data to 9DOF sensor//
void setCal() {

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

