#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "NMEAGPS.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 


NMEAGPS          gps; //create gps object
gps_fix          fix; // current GPS fix/ holds latest value
static const int RXPin = 4, TXPin = 3;
SoftwareSerial ss(RXPin, TXPin);
int z=1,a=0;
//change home position and boundary radius HERE!!
NeoGPS::Location_t home( 58039880L, 1006621890L ); // home position (degrees * 10,000,000)
const float thresholdDistance = 0.030;       // boundary radius (in km)

uint8_t           gpsSeconds; // for counting elapsed time instead of using delay

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xE6, 0xCD, 0x00, 0xCE, 0xB6, 0xA1, 0xB9, 0x8C, 0x40, 0xBE, 0x81, 0x08, 0x5F, 0xE9, 0x26, 0xD5 };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x0A, 0xA7, 0xF3, 0xD4, 0x08, 0x4F, 0xFC, 0x61, 0x68, 0xD1, 0xE5, 0x76, 0x10, 0xDA, 0x38, 0xE3 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x2604197D;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t unknown[]="INVALID";
static uint8_t inside[]="IN";
static uint8_t outside[]="OUT";
static osjob_t initjob,sendjob,blinkjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};
void do_send(osjob_t* j){
  byte mydata[2];
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else
    {
    while(z==1){
    while (gps.available( ss ))
      {
        fix = gps.read(); // read and save the latest gps value

    // Instead of delay, count the number of GPS fixes
    // check subject position for every gps read three times
    gpsSeconds++;
    if (gpsSeconds >= 3)
    {
      gpsSeconds = 0;
      Serial.println("8");

      displayInfo();
      checkDist();
      if (a==1)
      LMIC_setTxData2(1, inside, sizeof(inside)-1, 0);
      else if (a==2)
      LMIC_setTxData2(1, outside, sizeof(outside)-1, 0);
      else
      LMIC_setTxData2(1, unknown, sizeof(unknown)-1, 0);
      Serial.println("Packet queued");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Frequency: ");
      lcd.setCursor(0,1);
      lcd.print(LMIC.freq);
      delay(1000);
      z=0;
    }
    break;
      }
  }      
        // Prepare upstream data transmission at the next possible time.      
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(ev);
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print("Data Received: ");
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            z=1;
            break;
    }
}

void setup() {
    Serial.begin(9600);
    ss.begin(9600);
    lcd.begin(16,2);//Defining 16 columns and 2 rows of lcd display
    lcd.backlight();//To Power ON the back light
    while(!Serial);
    Serial.println("Starting");
    lcd.clear();
    lcd.print("Geofencing");
    delay(2000);
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    
    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
    
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

void displayInfo()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Position: ");
  delay(1000);

  if (fix.valid.location)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(fix.latitude(), 6);// print latitude
    lcd.setCursor(0,1);
    lcd.print(fix.longitude(), 6); // print longitude
    delay(2000);
  } 
  else
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("INVALID"); //ps: could be unclear sky
    delay(1000);
    a=0;
  }

} //end displayInfo()

void checkDist()
{
  if (fix.valid.location)
  {
    float dist = fix.location.DistanceKm( home ); //distance difference between subject and home position
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance:");
    lcd.setCursor(0,1);
    lcd.print(dist*1000, 6);
    lcd.print(" meters");
    delay(2000);

    if (dist >= thresholdDistance )
    { 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print((dist - thresholdDistance)*1000, 6); //distance difference between subject and boundary
      lcd.print(" meters");
      lcd.setCursor(0,1);
      lcd.print("beyond boundary");
      delay(2000);
      a=2;
    }
    else
    { 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Subject within");
      lcd.setCursor(0,1);
      lcd.print("boundary");
      delay(3000);
      a=1;
    }
  }
}
