#ifndef CONFIG_H
#define CONFIG_H

/**
 * exponential
 * averageNonBlocking
 * lowPass <3
 */
// const char *CONFIG = R"(
// {
//   "microcontroller": "teensy",
//   "midiCommunicationType": "serial",
//   "sensors": [
//     {
//       "sensorType": "potentiometer",
//       "messageType": "controlChange",
//       "controllerNumber": 112,
//       "statusCode": 176,
//       "inputPin": 14,
//       "intPin": 0,
//       "floorThreshold": 50,
//       "ceilThreshold": 950,
//       "filter": {
//         "type": "lowPass",
//         "weight": 2
//       }
//     }
//   ],
//   "uartConfig": [
//     {
//       "port": "Serial5",
//       "baudRate": 230400,
//       "communicationPurpose": "midi"
//     }
//   ]
// }
// )";

const char *CONFIG = R"(
{
  "microcontroller": "teensy",
  "midiCommunicationType": "serial",
  "sensors": [
    {
      "sensorType": "infrared",
      "messageType": "controlChange",
      "controllerNumber": 109,
      "statusCode": 176,
      "inputPin": 0,
      "intPin": 0,
      "floorThreshold": 30,
      "ceilThreshold": 375,
      "filter": {
        "type": "lowPass",
        "weight": 2
      },
      "modulator": {
        "target": "filterWeight",
        "minValue": 2,
        "maxValue": 10
      },
      "writeContinousValues": true
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
    },
    {
      "sensorType": "potentiometer",
      "messageType": "controlChange",
      "controllerNumber": 112,
      "statusCode": 176,
      "inputPin": 14,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 950,
      "filter": {
        "type": "lowPass",
        "weight": 2
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
