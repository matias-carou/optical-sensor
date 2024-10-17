#include "EspConfig.h"

const char *ESP32_CONFIG = R"(
{
  "sensors": [
    {
      "sensorType": "potentiometer",
      "messageType": "controlChange",
      "controllerNumber": 109,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 1000,
      "filter": {
        "type": "lowPass",
        "weight": 3
      }
    },
    {
      "sensorType": "potentiometer",
      "messageType": "controlChange",
      "controllerNumber": 110,
      "statusCode": 176,
      "inputPin": 1,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 1000,
      "filter": {
        "type": "lowPass",
        "weight": 3
      }
    },
    {
      "sensorType": "potentiometer",
      "messageType": "pitchBend",
      "controllerNumber": 111,
      "statusCode": 176,
      "inputPin": 3,
      "intPin": 0,
      "floorThreshold": 100,
      "ceilThreshold": 1000,
      "filter": {
        "type": "lowPass",
        "weight": 3
      }
    },
    {
      "sensorType": "accelgyro_ax",
      "messageType": "controlChange",
      "controllerNumber": 112,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 8,
      "floorThreshold": 400,
      "ceilThreshold": 15500,
      "filter": {
        "type": "lowPass",
        "weight": 5
      }
    },
    {
      "sensorType": "accelgyro_ay",
      "messageType": "controlChange",
      "controllerNumber": 113,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 7,
      "floorThreshold": 400,
      "ceilThreshold": 15500,
      "filter": {
        "type": "lowPass",
        "weight": 5
      }
    },
    {
      "sensorType": "infrared",
      "messageType": "controlChange",
      "controllerNumber": 114,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 10,
      "floorThreshold": 30,
      "ceilThreshold": 350,
      "filter": {
        "type": "lowPass",
        "weight": 1
      },
      "modulator": {
        "target": "filterWeight",
        "minValue": 2,
        "maxValue": 10
      },
      "writeContinousValues": true
    }
  ],
  "uartConfig": [
    {
      "port": "Serial",
      "baudRate": 230400,
      "communicationPurpose": "midi"
    }
  ]
}
)";