
#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MidiSensor.h"
#include "Config.h"

std::vector<MidiSensor *> SENSORS = {};

void setup() {
  delay(250);
  Serial.begin(9600);

  SENSORS = MidiSensor::initializeSensors(CONFIG);

  analogReadResolution(10);

  Serial.println(F("|| System ready <(':'<)\n"));
}

void loop() {
  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }
    SENSOR->run();
  }
  delayMicroseconds(500);
}