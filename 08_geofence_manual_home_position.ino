#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "NMEAGPS.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

NMEAGPS          gps; //create gps object
gps_fix          fix; // current GPS fix/ holds latest value

SoftwareSerial ss(4, 3);
int i;

//setup home position and boundary radius
NeoGPS::Location_t home;
bool got_home_pos = false;
const float        thresholdDistance = 0.030;       // boundary radius (in km)

uint8_t            gpsSeconds; // for counting elapsed time instead of using delay

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

void setup()
{
  ss.begin(9600);
  lcd.begin(16, 2); //Defining 16 columns and 2 rows of lcd display
  while (not got_home_pos) {
    if (gps.available( ss ))
    {
      fix = gps.read();
      if (fix.valid.location)
      {
        home = fix.location;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( home.latF(), 6);
        lcd.setCursor(0,1);
        lcd.print( home.lonF(), 6);
        delay(2000);
        got_home_pos = true;
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print( "No home position");
        delay(1000);
      }
    }
  }

} // setup

void loop()
{
  while (gps.available( ss ))
  {
    lcd.backlight();//To Power ON the back light
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
  }

} //end displayInfo()

void checkDist()
{
  if (fix.valid.location)
  {
    lcd.clear();
    float dist = fix.location.DistanceKm( home ); //distance difference between subject and home position
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
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Subject within");
      lcd.setCursor(0, 1);
      lcd.print("boundary");
      delay(3000);
    }
    delay(2000);
    lcd.clear();
    lcd.noBacklight();//To Power OFF the back light
    ss.end();

    for (i = 0; i < 1; i++)
    {
      //myWatchdogEnable (0b100000);  // 4 seconds
      myWatchdogEnable (0b100001);  // 8 seconds
    }
    ss.begin(9600);
  }
}
