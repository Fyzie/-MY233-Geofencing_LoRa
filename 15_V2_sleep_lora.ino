/*
  This sketch is to check current GPS coordinate against stored coordinates, display distance in km between them and check if the current location is within boundary (km)
*/
#include <Arduino.h>
#include <TinyGPS++.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <axp20x.h>
#include "SSD1306Wire.h" //for OLED display


TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;

#define I2C_SDA 21 //for OLED display
#define I2C_SCL 22 //for OLED display

// deep sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL); //for OLED display

#define SEND_BY_BUTTON 1

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

static uint8_t unknown[] = "INVALID";
static uint8_t incon[] = "CONNECTED:IN";
static uint8_t outcon[] = "CONNECTED:OUT";
static uint8_t indis[] = "DISCONNECTED:IN";
static uint8_t outdis[] = "DISCONNECTED:OUT";
static osjob_t initjob, sendjob, blinkjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},
};

int FuseinPin = 13;    // Security fuse connected to digital pin 13 --> connect to 5V & INPUT_PULLDOWN
bool Fuse_state = 0;      // variable to store the read value
int start = 1, geofence = 0, fuse = 0;
int i; int flag_TXCOMPLETE = 0;
bool result = false;

// *********************** Home-position & boundary definition ***************************************

typedef struct {
  float HOME_LAT;
  float HOME_LNG;
} mylocate;

RTC_DATA_ATTR mylocate locate;


bool got_home_pos = false;
const double boundary = 0.03; //define boundary of X km

// ***************************************************************************************************


void setup()
{
  Serial.begin(115200); //Baud rate 115200 for serial monitor

  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  GPS.begin(9600, SERIAL_8N1, 34, 12);   //TTGO ver1_1 TX and RX pin

  initOLED();

  pinMode(FuseinPin, INPUT_PULLDOWN);    // sets the digital pin 13 as input

  if (bootCount < 1) {
    while (not got_home_pos)
    {
      if (GPS.available())
      {
        if (gps.encode(GPS.read()) && gps.location.isValid())
        {
          locate.HOME_LAT = gps.location.lat();
          locate.HOME_LNG = gps.location.lng();
          display.clear();
          display.setTextAlignment(TEXT_ALIGN_LEFT);
          display.setFont(ArialMT_Plain_10);
          display.drawString(0, 0, "Home position");
          display.drawString(0, 10, "Latitude:");
          display.drawString(55, 10, String(locate.HOME_LAT, 6));
          display.drawString(0, 20, "Longitude:");
          display.drawString(55, 20, String(locate.HOME_LNG, 6));
          display.display();
          delay(2000);
          got_home_pos = true;
        }
        else
        {
          display.clear();
          display.setTextAlignment(TEXT_ALIGN_LEFT);
          display.setFont(ArialMT_Plain_10);
          display.drawString(0, 0, "No home position");
          display.display();
        }
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
    FIXED 1 FREQUENCY
    limit frequency transmitted by this lora node out of 64 channels
    all channel available: 902.3 - 914.9 MHz (902.3:0.2:914.9)
    chosen channel: 907.5 MHz (based on available nearest gateway)
  ***********************************************************************************************/
  // Disable all available channels
  /*for (int b = 0; b < 8; ++b) {
    LMIC_disableSubBand(b);
    }
    // Enable the channel(s) we want to use
    //LMIC_enableChannel(0); // 902.3 MHz
    LMIC_enableChannel(26);*/
  for (int channel = 0; channel < 26; ++channel) {
    LMIC_disableChannel(channel);
  }
  for (int channel = 27; channel < 64; ++channel) {
    LMIC_disableChannel(channel);
  }

  LMIC_setLinkCheckMode(0);

  LMIC.dn2Dr = DR_SF9;

  LMIC_setDrTxpow(DR_SF7, 14);
  while (not result)
  {
    while (GPS.available())
    {
      if (gps.encode(GPS.read()) && gps.location.isValid())
      {
        task();
        result = true;
        break;
      }
    }
  }
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  ++bootCount;
  display.resetDisplay();
  esp_deep_sleep_start();
}

void task()
{
  double distanceKM =
    gps.distanceBetween( //haversine distance between the current and home location
      gps.location.lat(),
      gps.location.lng(),
      locate.HOME_LAT,
      locate.HOME_LNG) / 1000.0; //distance in km

  smartDelay(2500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  security_fuse();

  displayOLED(gps.location.lat(), gps.location.lng(), distanceKM);

  while (flag_TXCOMPLETE == 0)
  {
    os_runloop_once();
  }
  flag_TXCOMPLETE = 0;
}
void loop() {

}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
  } while (millis() - start < ms);
}

void initOLED() {
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void displayOLED(double latitude, double longitude, double distanceKM) {
  display.clear();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Latitude:");
  display.drawString(55, 0, String(latitude, 5));
  display.drawString(0, 10, "Longitude:");
  display.drawString(55, 10, String(longitude, 5));

  display.setFont(ArialMT_Plain_10);
  //  display.drawString(0, 42, String(millis()));
  display.drawString(0, 20, "Distance (m):");
  display.drawString(75, 20, String(distanceKM * 1000, 2)); //distance in meters

  if (distanceKM > boundary) {
    display.drawString(0, 30, "Out of boundary");
    geofence = 2;
  }
  else {
    display.drawString(0, 30, "In boundary");
    geofence = 1;
  }

  if (Fuse_state == false)
  {
    display.drawString(0, 40, "Security fuse disconnected");
    fuse = 1;
  }

  if (Fuse_state == true)
  {
    display.drawString(0, 40, "Security fuse connected");
  }
  display.display();

  do_send(&sendjob);
}

void security_fuse()
{
  Fuse_state = digitalRead(FuseinPin);
}

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
  display.drawString(0, 50, "Frequency:");
  display.drawString(55, 50, String(LMIC.freq));
  display.display();
}

void onEvent (ev_t ev)
{
  flag_TXCOMPLETE = 1;
}
