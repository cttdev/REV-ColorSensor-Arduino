#include <Wire.h>

#include "REVColorSensor.h"

REVColorSensor sensor = REVColorSensor();

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.begin();
}

void loop()
{
  Serial.println("Red: " + String(sensor.getRed()) + " Green: " + String(sensor.getGreen()) + " Blue: " + String(sensor.getBlue()));
}