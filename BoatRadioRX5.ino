// rf69 demo tx rx.pde
// -*- mode: C++ -*-
/* Sketch to pack up the record received from the boat and send it to Cayenne
     brown - started program
      red - finished setup
      blue - waiting for GPS data
      green - successful send of data to TX board 
      purple - Port 
      yellow - Starboard 
      */
//#define CAYENNE_DEBUG
#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP8266.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF69.h>
//#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

// WiFi network info.
char ssid[] = "???????";
char wifiPassword[] = "?????????";

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
//char username[] = "????????????";
char password[] = "????????????????";
char clientID[] = "????????????";

// These are the commands sent from Cayenne to control the motor speed
//1500 is the pwm value for a stopped motor
int OverrideButton = 0;
int PortMotor = 1500;
int StarboardMotor = 1500;


typedef struct {
   char Recno1{2};
   char Date[7];
   char Time[7];
   char Latitude[11];
   char Longitude[11];
   int ActualCourse;
   int CalcCourse;
   float Speed;
   int Distance;
   }  RECORD1;


RECORD1 rec1;


// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_IRQ);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{

  Serial.begin(115200);
  Cayenne.begin(username, password, clientID, ssid, wifiPassword);
 while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


void loop() {

 Cayenne.loop();

 
  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    Serial.print("length = ");Serial.println(len);
    Serial.print("buf = ");Serial.println((char*)buf);
   if (rf69.recv(buf, &len)) {
     if (!len) return;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);
      
     
        UnpackRecord1(buf);
        SendToCayenne();
      }
     // if (buf[0] == '1' ) {
      
        // Send a reply!
        
          //   pack the reply to send back
          char temp[10];
          uint8_t data[] = "aaaaaaaaa";
          sprintf(temp, "%d%4d%4d",
                OverrideButton,
                PortMotor,
                StarboardMotor);
          for (int i=0;i<9; i++)
            data[i] = temp[i];
                
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        Serial.println("Sent a reply");
       
      //}
    } else {
      Serial.print("Receive failed");Serial.println(random(1,10000));
    }
   delay(10000);
  }
  
 




void SendToCayenne()
{

  //Publish data every 30 seconds (30000 milliseconds). Change this value to publish at a different interval.
//  if (millis() - lastMillis > 30000) {lastMillis = millis();}
    
  
   
 //Write data to Cayenne here.   
     Serial.print("Cayenne Latitude");Serial.println(rec1.Latitude);
    Cayenne.virtualWrite(1,  rec1.Latitude, "analog_sensor", "null");
    Cayenne.virtualWrite(2,  rec1.Longitude, "analog_sensor", "null");
    Cayenne.virtualWrite(3,  rec1.CalcCourse, "analog_sensor", "null");
    Cayenne.virtualWrite(4,  rec1.Speed, "wind_speed", "kmh");
    Cayenne.virtualWrite(5,  rec1.Distance, "prox", "m");
    Cayenne.virtualWrite(17,  rec1.ActualCourse, "analog_sensor", "null");
    
}

void UnpackRecord1(uint8_t *buf) {
  char temp[10];
    sprintf(rec1.Date,"%-6.6s", buf+1);
    sprintf(rec1.Time,"%-6.6s", buf+7);
    sprintf(rec1.Latitude,"%-9.9s", buf+13);
   // rec1.Latitude = atof(temp);
    sprintf(rec1.Longitude,"%-9.9s", buf+22);
  //  rec1.Longitude = atof(temp);
    sprintf(temp, "%-3.3s", buf+31);
    rec1.ActualCourse = atoi(temp);
    sprintf(temp, "%-3.3s", buf+34);
    rec1.CalcCourse = atoi(temp);
    sprintf(temp,"%-3.3s", buf+37);
    rec1.Speed = atof(temp);
    sprintf(temp,"%-5.5s", buf+40);
    rec1.Distance = atof(temp);

    Serial.println(rec1.Date);
    Serial.println(rec1.Time);
    Serial.println(rec1.Latitude);
    Serial.println(rec1.Longitude); 
    Serial.println(rec1.ActualCourse);  
    Serial.println(rec1.CalcCourse);   
    Serial.println(rec1.Speed);
    Serial.println(rec1.Distance);

   
    }  


// This function is called when data is sent from Cayenne.
CAYENNE_IN(18)
{
  OverrideButton = getValue.asInt();
 
  Serial.print("Virtual channel = "); Serial.println(18);
  Serial.print("Value = "); Serial.println(OverrideButton);
}
// This function is called when data is sent from Cayenne.
CAYENNE_IN(19)
{
  PortMotor = getValue.asInt();

  Serial.print("Virtual channel = "); Serial.println(19);
  Serial.print("Value = "); Serial.println(PortMotor);
}
// This function is called when data is sent from Cayenne.
CAYENNE_IN(20)
{
  StarboardMotor = getValue.asInt();

  Serial.print("Virtual channel = "); Serial.println(20);
  Serial.print("Value = "); Serial.println(StarboardMotor);
}



