#include "Config.h"

/**
 * exponential
 * averageNonBlocking
 * lowPass <3
 */
const char *CONFIG = R"(
{
  "midiCommunicationType": "ble",
  "sensors": [
    {
      "sensorType": "potentiometer",
      "messageType": "controlChange",
      "controllerNumber": 109,
      "statusCode": 176,
      "inputPin": 4,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 980,
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
      "controllerNumber": 111,
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
      "controllerNumber": 112,
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