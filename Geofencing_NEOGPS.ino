#include <Arduino.h>
#include "NMEAGPS.h"
#include <SoftwareSerial.h>

NMEAGPS          gps;
gps_fix          fix; // current GPS fix/ holds latest value
static const int RXPin = 4, TXPin = 3;
SoftwareSerial ss(RXPin, TXPin);

//change home position and boundary radius HERE!!
NeoGPS::Location_t home( 58039880L, 1006621890L ); // home position (degrees * 10,000,000)
const float        thresholdDistance = 0.022;       // boundary radius (in km)

uint8_t            gpsSeconds; // for counting elapsed time instead of using delay

void setup()
{
  Serial.begin(9600);
  Serial.println("Geofencing started!");

  ss.begin(9600);

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
  Serial.print("Position: ");

  if (fix.valid.location)
  {
    Serial.print("lat: "); Serial.print(fix.latitude(), 6); Serial.print(" ");// print latitude
    Serial.print("lon: "); Serial.println(fix.longitude(), 6); // print longitude
  } else
    Serial.println( '?' ); //ps: could be unclear sky

} //end displayInfo()

void checkDist()
{
  if (fix.valid.location)
  {
    float dist = fix.location.DistanceKm( home ); //distance difference between subject and home position
    Serial.print("Distance between subject and home position:");
    Serial.print(dist*1000, 6);
    Serial.println(" meters");

    if (dist >= thresholdDistance )
    {
      Serial.print("Subject is ");
      Serial.print(dist - thresholdDistance, 6); //distance difference between subject and boundary
      Serial.println("KM beyond boundary");
    }
    else
    {
      Serial.println("Subject within boundary, no worries!");
    }
  }
}
