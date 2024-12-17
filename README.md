## A Wireless, Rechargeable MIDI Controller Designed with C++, Arduino and PlatformIO
<p align="center">
  <img src="https://github.com/user-attachments/assets/b4340b90-cb64-4564-965a-65739d3961ce" alt="ble_controller_inst" width="600"/>
</p>

### Microcontroller
    - ESP32-C3
### Sensors
    - 3 Potentiometers
    - 2 Sensors
      - VL530X
      - MPU6050
    - 3 Switches
      - AX
      - AY
      - Distance
    - 1 RST Button
### Base Config
```
'{
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
      "inputPin": 4,
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
      "inputPin": 1,
      "intPin": 0,
      "floorThreshold": 50,
      "ceilThreshold": 950,
      "filter": {
        "type": "lowPass",
        "weight": 1
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
      "floorThreshold": 50,
      "ceilThreshold": 325,
      "filter": {
        "type": "lowPass",
        "weight": 1
      },
      "modulator": {
        "target": "filterWeight",
        "minValue": 2,
        "maxValue": 10
      }
    }
  ]
}'
```

### Macros
  - Define the microcontroller to be used
    - __MICROCONTROLLER_ESP32__: Serial Communication
    - __MICROCONTROLLER_TEENSY__: ESP32: BLE Server
```
#define MICROCONTROLLER MICROCONTROLLER_ESP32
// #define MICROCONTROLLER MICROCONTROLLER_TEENSY
```

### Communication Client (Optional)
- The codebase supports both BLE (ESP32) and Serial (Tested with XBee series 3)
  - A BLE client will be setup as default with guardrails to avoid initialization with a non ESP32 micro
```
'{
  "uartConfig": [
    {
      "port": "Serial",
      "baudRate": 230400,
      "communicationPurpose": "midi"
    }
  ]
}'
```

