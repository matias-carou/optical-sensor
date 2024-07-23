
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
        "weight": 2,
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

std::vector<MidiSensor *> SENSORS = {};

void setup() {
  delay(250);
  Serial.begin(230400);

  Serial.println(F("|| Starting program..."));
  Serial.println(F("|| Setting up sensors..."));

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
  delayMicroseconds(250);
}