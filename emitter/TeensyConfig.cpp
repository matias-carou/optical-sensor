#include "TeensyConfig.h"

const char *TEENSY_CONFIG = R"(
{
  "sensors": [
    {
      "sensorType": "potentiometer",
      "messageType": "controlChange",
      "controllerNumber": 109,
      "statusCode": 176,
      "inputPin": 14,
      "intPin": 0,
      "floorThreshold": 100,
      "ceilThreshold": 950,
      "filter": {
        "type": "lowPass",
        "weight": 4
      }
    },
    {
      "sensorType": "accelgyro_ax",
      "messageType": "controlChange",
      "controllerNumber": 110,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 2,
      "floorThreshold": 400,
      "ceilThreshold": 15500,
      "filter": {
        "type": "lowPass",
        "weight": 4
      }
    },
    {
      "sensorType": "accelgyro_ay",
      "messageType": "controlChange",
      "controllerNumber": 111,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 3,
      "floorThreshold": 400,
      "ceilThreshold": 15500,
      "filter": {
        "type": "lowPass",
        "weight": 4
      }
    },
    {
      "sensorType": "infrared",
      "messageType": "controlChange",
      "controllerNumber": 112,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 4,
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
      "port": "Serial5",
      "baudRate": 230400,
      "communicationPurpose": "midi"
    }
  ]
}
)";