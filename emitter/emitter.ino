#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MidiSensor.h"
#include "Wire.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const char *CONFIG = R"(
{
  "microcontroller": "teensy",
  "sensors": [
    {
      "sensorType": "infrared",
      "controllerNumber": 109,
      "pin": 0,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 400,
      "messageType": "controlChange",
      "filter": {
        "amountOfReads": 2,
        "filterType": "averageNonBlocking"
      }
    }
  ],
  "uartConfig": [
    {
      "port": "Serial",
      "baudRate": 230400
    },
    {
      "port": "Serial5",
      "baudRate": 230400
    }
  ]
}
)";

std::vector<MidiSensor *> SENSORS = MidiSensor::initializeSensors(CONFIG);

void setup() {
  delay(250);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  analogReadResolution(10);

  Serial.println("Setting up pins...");

  MidiSensor::setUpSensorPins(SENSORS);

  Serial.println("System ready <(':'<)");
}

void loop() {
  for (MidiSensor *SENSOR : SENSORS) {
    SENSOR->run(&lox);
  }
  delayMicroseconds(500);
}