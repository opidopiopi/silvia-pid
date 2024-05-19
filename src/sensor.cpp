#include <shared.h>
#include <sensor.h>
#include <Adafruit_MAX31865.h>

#include <ZACwire.h>


ZACwire boilerTemp(D6);
ZACwire brewHeadTemp(D7);

const double factor = (3.3/1024.0) * 100.0;

double currentTempReadBoiler = 0.0;
double currentTempReadBrewhead = 0.0;

void setupSensor(){
  if (boilerTemp.begin() == true)
  {
    Serial.println("Boiler Sensor online");
  }
  delay(3);
  if (brewHeadTemp.begin() == true)
  {
    Serial.println("Brewhead Sensor online");
  }
  delay(3);
}


unsigned long lastMeasurement = 0;

double getBoilerTemperature(){
    currentTempReadBoiler = boilerTemp.getTemp();

    return currentTempReadBoiler;
}

double getBrewHeadTemperature(){
    currentTempReadBrewhead = brewHeadTemp.getTemp();

    return currentTempReadBrewhead;
}