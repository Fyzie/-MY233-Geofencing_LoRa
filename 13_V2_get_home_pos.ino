/*
  This sketch is to check current GPS coordinate against stored coordinates, display distance in km between them and check if the current location is within boundary (km)
*/
#include <Arduino.h>
#include <TinyGPS++.h>
#include <axp20x.h>
#include "SSD1306Wire.h" //for OLED display

TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;

#define I2C_SDA 21 //for OLED display
#define I2C_SCL 22 //for OLED display

SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL); //for OLED display

int FuseinPin = 13;    // Security fuse connected to digital pin 13 --> connect to 5V & INPUT_PULLDOWN
bool Fuse_state = 0;      // variable to store the read value
bool detection;

// *********************** Co-ordinates & boundary definition ***************************************

double HOME_LAT;
double HOME_LNG;

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

  while (not got_home_pos)
  {
    if (GPS.available())
    {
      if (gps.encode(GPS.read()) && gps.location.isValid())
      {
        HOME_LAT = gps.location.lat();
        HOME_LNG = gps.location.lng();
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, "Home position");
        display.drawString(0, 10, "Latitude:");
        display.drawString(55, 10, String(HOME_LAT, 6));
        display.drawString(0, 20, "Longitude:");
        display.drawString(55, 20, String(HOME_LNG, 6));
        display.display();
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

void loop()
{
  double distanceKM =
    gps.distanceBetween( //haversine distance between the current and home location
      gps.location.lat(),
      gps.location.lng(),
      HOME_LAT,
      HOME_LNG) / 1000.0; //distance in km

  smartDelay(2500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  security_fuse();

  displayOLED(gps.location.lat(), gps.location.lng(), distanceKM);

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
  }
  else {
    display.drawString(0, 30, "In boundary");
  }

  if (Fuse_state == false)
  {
    display.drawString(0, 40, "Security fuse disconnected");
  }

  if (Fuse_state == true)
  {
    display.drawString(0, 40, "Security fuse connected");
  }

  display.display();
}

void security_fuse()
{
  Fuse_state = digitalRead(FuseinPin);
}
