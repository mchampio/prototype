
#include <SPI.h>
#include <RH_RF69.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timeb.h> // truct timeb, time(&t) for msecs
#include <time.h>      // struct tm, time(&t) to secs
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

/*
   This sketch will use the take the record of gps etc data and send it from the boat via radio to shore
  

   The neopixel will be used to show the status of the board
      brown - started program
      red - finished setup
      blue - data successfully sent to shore
      green - data successfuly received from shore
      purple - data successfuly sent to navigate cpu
      yellow - data successfuly received from navigate cpu
*/


/************ Radio Setup ***************/

// Change to 434.0 
#define RF69_FREQ 434.0

#define RFM69_CS      10   // "B"
#define RFM69_RST     11   // "A"
#define RFM69_IRQ     6    // "D"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )

#define LED 13             // pin 13 is the LED 
// define the pin for the Neopixel
#define PIN            8
// How many NeoPixels are attached to the Arduino?, set up Neopixels
#define NUMPIXELS      1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int OverrideButton = 0;
int PortMotor = 1500;
int StarboardMotor = 1500;

#define SLAVE_ADDRESS 0x60       
byte x = 0;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = "015001500";
char buf1[60];
uint8_t len = sizeof(buf);



// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_IRQ);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  pixels.begin(); // This initializes the NeoPixel library.
  Neopixel(25, 13, 0); //brown
delay(3000);
        
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(115200);
 //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(13, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
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

    Neopixel(45, 0, 0); //red
}


void loop() {


  // Send Record 1
  char buf1[60] = {0,1,2,3,4,5,6,7,8,9,0,0,1,2,3,4,5,6,7,8,9,0,0,1,2,3,4,5,6,7,8,9,0,0,1,2,3,4,5,6,7,8,9,0,0,1,2,3,4,5,6,7,8,9,0,0,1,2,0};
 // int myArray[10]={9,3,2,4,3,2,7,8,9,11};
  rf69.send((uint8_t *)buf1, sizeof(buf1));
  rf69.waitPacketSent();
 Serial.print("Sent record 1 "); Serial.println(buf1); 
 Serial.print("length = "); Serial.println(sizeof(buf1));
    Neopixel(0, 39, 64); //blue
  //  wait for .5 sec to allow receiver to catch up
   delay(500);
   
  
  // Now wait for a reply
  
  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      //UnpackReply(buf);
      Neopixel(0, 64, 0); //green  
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }
  delay(10000);  // Wait 10 second between transmits, could also 'sleep' here!
}


void UnpackReply(uint8_t *buf) {
    char temp[10];
    sprintf(temp,"%-1.1s", buf); OverrideButton = atof(temp);
    sprintf(temp,"%-4.4s", buf+1); PortMotor = atof(temp);
    sprintf(temp,"%-4.4s", buf+5); StarboardMotor = atof(temp);
    
    Serial.print("OverrideButton = "); Serial.println(OverrideButton);
    Serial.print("PortMotor = "); Serial.println(PortMotor);
    Serial.print("StarboardMotor = "); Serial.println(StarboardMotor);

    }  


void requestEvent() 
{
  Serial.print("Request from Master. Sending: ");
  Serial.print((char*)buf);
  Serial.print(" len ");
  Serial.print(len);

  Wire.write(buf,len);
  Neopixel(64, 0, 64);    //purple
}

void receiveEvent(int bytes)
{
   Serial.print("Data from Master. Received ");
  if(Wire.available() != 0)
  {
   
      for(int i = 0; i< bytes; i++)
       {
        buf1[i] = Wire.read();
       }  
     buf1[bytes] = '\0';
     Neopixel(64, 64, 0); //yellow
     Serial.print("buf1 = ");
     
    Serial.println(buf1);
    }
  }

void Neopixel(int r, int g, int b) {
    Serial.print(" r=");Serial.print(r);Serial.print(" g=");Serial.print(g);  Serial.print(" b=");Serial.println(b); 
       // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(0, pixels.Color(r,g,b)); 

    pixels.show(); // This sends the updated pixel color to the hardware.

    }  
