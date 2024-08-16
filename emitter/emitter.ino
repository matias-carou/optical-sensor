
#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MidiSensor.h"

std::vector<MidiSensor *> SENSORS = {};

const int RESET_PIN = 2;

void setup() {
  Serial.begin(9600);

  SENSORS = MidiSensor::initializeSensors(CONFIG);
  pinMode(RESET_PIN, INPUT_PULLUP);

  analogReadResolution(10);

  Serial.println(F("|| System ready <(':'<)\n"));
}

void loop() {
  if (!digitalRead(RESET_PIN)) {
    SCB_AIRCR = 0x05FA0004;
  }

  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }
    SENSOR->run();
  }
  delayMicroseconds(500);
}