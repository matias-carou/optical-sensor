#ifndef CONFIG_H
#define CONFIG_H

/**
 * exponential
 * averageNonBlocking
 * lowPass
 */
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
        "type": "lowPass",
        "weight": 2
      },
      "communicationType": "continous"
    },
    {
      "sensorType": "accelgyro_ax",
      "messageType": "controlChange",
      "controllerNumber": 110,
      "statusCode": 176, 
      "inputPin": 0,
      "intPin": 0,
      "floorThreshold": 1000,
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
      "intPin": 0,
      "floorThreshold": 1000,
      "ceilThreshold": 15500,
      "filter": {
        "type": "lowPass",
        "weight": 4
      }
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

#endif
