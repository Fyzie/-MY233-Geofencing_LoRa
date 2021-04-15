/**************************************************************************
 MY233
 MUHAMMAD HAFIZI BIN ABDUL MALIK
 GEOFENCING LOCATION TRACKING SYSTEM USING IOT AND LORA LPWAN FOR COVID19
 MANDATORY SELF QUARANTINE MONITORING
 *************************************************************************/
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "NMEAGPS.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define VCC2 8

NMEAGPS          gps; //create gps object
gps_fix          fix; // current GPS fix/ holds latest value
static const int RXPin = 4, TXPin = 3;
SoftwareSerial ss(RXPin, TXPin);

NeoGPS::Location_t home;
bool got_home_pos = false;

/***********************************************************************************************
 GEOFENCE BOUNDARY
 edit boundary HERE!!
 geofence boundary can be adjusted to cater for different quarantine situations
***********************************************************************************************/
const float thresholdDistance = 0.020;       // boundary radius (in km)

uint8_t           gpsSeconds; // for counting elapsed time instead of using delay

/***********************************************************************************************
REGISTERED LORA NODE ON THE THINGS NETWORK (TTN)
***********************************************************************************************/
// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x5B, 0xE2, 0x7B, 0x83, 0x6D, 0x14, 0x44, 0xE9, 0x80, 0x07, 0xB0, 0x3F, 0x4C, 0x26, 0x38, 0xE1 };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xB4, 0x58, 0x10, 0xD8, 0xB9, 0x63, 0xB3, 0xD7, 0x6F, 0x51, 0x6B, 0x66, 0x35, 0xD0, 0x6E, 0x17 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26041E2A;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/***********************************************************************************************
 LIST OF POSSIBLE TRANSMITTED INFO
***********************************************************************************************/
static uint8_t unknown[] = "INVALID";
static uint8_t incon[] = "CONNECTED:IN";
static uint8_t outcon[] = "CONNECTED:OUT";
static uint8_t indis[] = "DISCONNECTED:IN";
static uint8_t outdis[] = "DISCONNECTED:OUT";
static osjob_t initjob, sendjob, blinkjob;

int FuseinPin = 7;    // pushbutton connected to digital pin 7
bool Fuse_state = 0;      // variable to store the fuse-read value
bool detection;
int geofence = 0, fuse = 0;
int i; int flag_TXCOMPLETE = 0;

//const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};
/***********************************************************************************************
 STARTING DATA TRANSMISSION
***********************************************************************************************/
void do_send(osjob_t* j)
{
  if (geofence == 1 && fuse == 0)
    LMIC_setTxData2(1, incon, sizeof(incon) - 1, 0);
  else if (geofence == 2 && fuse == 0)
    LMIC_setTxData2(1, outcon, sizeof(outcon) - 1, 0);
  else if (geofence == 1 && fuse == 1)
    LMIC_setTxData2(1, indis, sizeof(indis) - 1, 0);
  else if (geofence == 2 && fuse == 1)
    LMIC_setTxData2(1, outdis, sizeof(outdis) - 1, 0);
  else
    LMIC_setTxData2(1, unknown, sizeof(unknown) - 1, 0);
    //debugging
    /*lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Frequency: ");
    lcd.setCursor(0, 1);
    lcd.print(LMIC.freq);
    delay(1000);*/
}
/***********************************************************************************************
 SUCCESSFUL DATA TRANSMISSION TO THE GATEWAY
***********************************************************************************************/
void onEvent (ev_t ev)
{
  flag_TXCOMPLETE = 1;
  //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
}
/***********************************************************************************************
 SLEEP/ LOW POWER MODE
***********************************************************************************************/
ISR(WDT_vect)
{
  wdt_disable();  // disable watchdog
}

void myWatchdogEnable(const byte interval)
{
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay

  wdt_reset();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_mode();            // now goes to Sleep and waits for the interrupt
}
/***********************************************************************************************
 MAIN
***********************************************************************************************/
void setup()
{
  ss.begin(9600);
  pinMode(VCC2,OUTPUT);
  digitalWrite(VCC2, HIGH);
  pinMode(FuseinPin, INPUT);    // sets the digital pin 7 as input
  lcd.begin(16, 2); //Defining 16 columns and 2 rows of lcd display
  lcd.backlight();//To Power ON the back light
  lcd.clear();
  lcd.print("Geofencing");
  delay(2000);
/***********************************************************************************************
 HOME POSITION
 upon switch on by the user, home position will be set
 make sure the user stay in the middle of home during the process
***********************************************************************************************/
  while (not got_home_pos) {
    if (gps.available( ss ))
    {
      fix = gps.read();
      if (fix.valid.location)
      {
        home = fix.location;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print( home.latF(), 6);
        lcd.setCursor(0, 1);
        lcd.print( home.lonF(), 6);
        delay(2000);
        got_home_pos = true;
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print( "No home position");
        delay(1000);
      }
    }
  }
#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  os_init();
  LMIC_reset();
#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

/***********************************************************************************************
 FIXED FREQUENCY
 limit frequency transmitted by this lora node out of 64 channels
 all channel available: 902.3 - 914.9 MHz (902.3:0.2:914.9)
 chosen channel: 907.5 MHz (based on available nearest gateway)
***********************************************************************************************/
  for (int channel = 0; channel < 26; ++channel) {
    LMIC_disableChannel(channel);
  }
  for (int channel = 27; channel < 64; ++channel) {
    LMIC_disableChannel(channel);
  }

  LMIC_setLinkCheckMode(0);

  LMIC.dn2Dr = DR_SF9;

  LMIC_setDrTxpow(DR_SF7, 14);
}

void loop()
{
/***********************************************************************************************
 GPS READING
***********************************************************************************************/
  while (gps.available( ss ))
    {
      lcd.backlight();//To Power ON the back light
      fix = gps.read(); // read and save the latest gps value

      // Instead of delay, count the number of GPS fixes
      // check subject position for every gps read three times
      gpsSeconds++;

      security_fuse();

      if (gpsSeconds >= 3)
      {
        gpsSeconds = 0;
        displayInfo();
        checkDist();
        do_send(&sendjob);
/***********************************************************************************************
WAITING FOR SUCCESSFUL DATA TRANSMISSION
start loop when data available
end loop once data transmission is successful
***********************************************************************************************/
        while (flag_TXCOMPLETE == 0)
        {
          os_runloop_once();
        }
        flag_TXCOMPLETE = 0;
          
        lcd.clear();
        lcd.noBacklight();//To Power OFF the back light
        ss.end();
/***********************************************************************************************
GOING TO SLEEP MODE
sleep time can be adjusted (i = 0; i < XXX; i++)
current experimental setup: (15*8 seconds) minutes == 2 minutes
***********************************************************************************************/
        for (i = 0; i < 15; i++)
        {
          myWatchdogEnable (0b100001);  // 8 seconds
          //myWatchdogEnable (0b100000);  // 4 seconds
        }
        ss.begin(9600);
      }
    }
}

/***********************************************************************************************
USER LOCATION DEBUGGING
***********************************************************************************************/
void displayInfo()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Position: ");
  delay(1000);

  if (fix.valid.location)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(fix.latitude(), 6);// print latitude
    lcd.setCursor(0, 1);
    lcd.print(fix.longitude(), 6); // print longitude
    delay(2000);
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("INVALID"); //ps: could be unclear sky
    delay(1000);
    geofence = 0;
  }

} //end displayInfo()
/***********************************************************************************************
GEOFENCE CALCULATION
***********************************************************************************************/
void checkDist()
{
  if (fix.valid.location)
  {
    float dist = fix.location.DistanceKm( home ); //distance difference between subject and home position
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance:");
    lcd.setCursor(0, 1);
    lcd.print(dist * 1000, 6);
    lcd.print(" meters");
    delay(2000);

    if (dist >= thresholdDistance )
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print((dist - thresholdDistance) * 1000, 6); //distance difference between subject and boundary
      lcd.print(" meters");
      lcd.setCursor(0, 1);
      lcd.print("beyond boundary");
      delay(2000);
      geofence = 2;
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Subject within");
      lcd.setCursor(0, 1);
      lcd.print("boundary");
      delay(3000);
      geofence = 1;
    }
  }
}
/***********************************************************************************************
 SECURITY FUSE
 once the fuse is disconnected, it will never be connected again by the user
***********************************************************************************************/
void security_fuse()
{
  Fuse_state = digitalRead(FuseinPin);

  if (Fuse_state == false) //Checks status of security fuse is disconnected
    detection = true;

  if (detection == true)
  {
    fuse = 1;
  }
}
