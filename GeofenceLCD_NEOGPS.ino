#include <Arduino.h>
#include "NMEAGPS.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

NMEAGPS          gps; //create gps object
gps_fix          fix; // current GPS fix/ holds latest value
static const int RXPin = 4, TXPin = 3;
SoftwareSerial ss(RXPin, TXPin);

//change home position and boundary radius HERE!!
NeoGPS::Location_t home( 58039880L, 1006621890L ); // home position (degrees * 10,000,000)
const float        thresholdDistance = 0.030;       // boundary radius (in km)

uint8_t            gpsSeconds; // for counting elapsed time instead of using delay

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  lcd.begin(16,2);//Defining 16 columns and 2 rows of lcd display
  lcd.backlight();//To Power ON the back light
  lcd.print("Starting");

} // setup

void loop()
{
  while (gps.available( ss ))
  {
    fix = gps.read(); // read and save the latest gps value

    // Instead of delay, count the number of GPS fixes
    // check subject position for every gps read three times
    gpsSeconds++;
    if (gpsSeconds >= 3)
    {
      gpsSeconds = 0;

      displayInfo();
      checkDist();
    }
  }
} // loop

void displayInfo()
{
  lcd.setCursor(0,0);
  lcd.print("Position: ");
  delay(1000);
  lcd.clear();

  if (fix.valid.location)
  {
    lcd.setCursor(0,0);
    lcd.print(fix.latitude(), 6);// print latitude
    lcd.setCursor(0,1);
    lcd.print(fix.longitude(), 6); // print longitude
    delay(2000);
    lcd.clear();
  } 
  else
  {
    lcd.setCursor(0,0);
    lcd.print("INVALID"); //ps: could be unclear sky
    delay(1000);
    lcd.clear();
  }

} //end displayInfo()

void checkDist()
{
  if (fix.valid.location)
  {
    float dist = fix.location.DistanceKm( home ); //distance difference between subject and home position
    lcd.setCursor(0,0);
    lcd.print("Distance:");
    lcd.setCursor(0,1);
    lcd.print(dist*1000, 6);
    lcd.print(" meters");
    delay(2000);
    lcd.clear();

    if (dist >= thresholdDistance )
    { 
      lcd.setCursor(0,0);
      lcd.print((dist - thresholdDistance)*1000, 6); //distance difference between subject and boundary
      lcd.print(" meters");
      lcd.setCursor(0,1);
      lcd.print("beyond boundary");
      delay(2000);
      lcd.clear();
    }
    else
    { 
      lcd.setCursor(0,0);
      lcd.print("Subject within");
      lcd.setCursor(0,1);
      lcd.print("boundary");
      delay(3000);
      lcd.clear();
    }
  }
}
