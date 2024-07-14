#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"

// MPU6050 accelgyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const char* CONFIG = R"(
{
  "sensors": [
    {
      "sensorType": "infrared",
      "controllerNumber": 109,
      "pin": 0,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 450,
      "messageType": "controlChange",
      "filter": {
        "amountOfReads": 1,
        "filterType": "averageNonBlocking"
      }
    }
  ],
  "hardwareSerialBaudRate": 230400
}
)";

std::vector<Sensor*>
  SENSORS = Sensor::initializeSensors(CONFIG);

void setup() {
  delay(250);
  Serial5.begin(230400);

  Serial.println("Initializing I2C sensors...");

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }

  analogReadResolution(10);

  Serial.println("Setting up pins...");

  Sensor::setUpSensorPins(SENSORS);

  Serial.println("System ready <(':'<)");
}

const static std::vector<std::string> SIBLINGS = { "ax", "ay" };

void loop() {
  // const uint8_t activeSiblings = Sensor::getActiveSiblings(SENSORS, SIBLINGS);
  // const uint8_t areAllSiblingsDebounced = Sensor::areAllSiblingsDebounced(SENSORS, SIBLINGS);
  for (Sensor* SENSOR : SENSORS) {
    // const bool isSiblingButWithPendingDebounce = SENSOR->isSibling(SIBLINGS) && !areAllSiblingsDebounced;
    // if (!SENSOR->isSwitchActive()) {
    //   continue;
    // }
    SENSOR->run(&lox);
  }
  delayMicroseconds(500);
}