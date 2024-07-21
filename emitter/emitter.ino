
#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MidiSensor.h"
#include "Wire.h"

const char *CONFIG = R"(
{
  "microcontroller": "teensy",
  "sensors": [
    {
      "sensorType": "infrared",
      "messageType": "controlChange",
      "controllerNumber": 109,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 0,
      "floorThreshold": 30,
      "ceilThreshold": 350,
      "filter": {
        "filterType": "lowPassFilter",
        "amountOfReads": 2
      },
      "communicationType": "continous"
    }
  ],
  "uartConfig": [
    {
      "port": "Serial5",
      "baudRate": 230400,
      "communicationPurpose": "midi"
    }
  ]
}
)";

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
std::vector<MidiSensor *> SENSORS = {};

void setup() {
  delay(250);
  Serial.begin(230400);

  Serial.println("|| Starting program...");
  Serial.println("|| Setting up sensors...");

  SENSORS = MidiSensor::initializeSensors(CONFIG);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  analogReadResolution(10);

  Serial.println("|| System ready <(':'<)\n");
}

void loop() {
  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }
    SENSOR->run(&lox);
  }
  delayMicroseconds(500);
}