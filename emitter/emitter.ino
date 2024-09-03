/**
 * arduino-cli compile --fqbn esp32:esp32:esp32c3 . && arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn esp32:esp32:esp32c3 . &&
 * arduino-cli monitor -p /dev/cu.usbmodem2101
 */

#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "I2Cdev.h"
#include "MidiSensor.h"
#include "Utils.h"

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
#include <BLEMidi.h>
#endif

std::vector<MidiSensor *> SENSORS = {};

const int LED_PIN = 3;

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
const int RESET_PIN = 2;
#endif

void setup() {
  Serial.begin(9600);

  const std::string microControllerValue = Utils::getMicrocontrollerReadableValue();

  const std::string str = "|| Uploading code for microcontroller " + microControllerValue;
  Serial.println(str.c_str());

  SENSORS = MidiSensor::initializeSensors(CONFIG);

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
  pinMode(RESET_PIN, INPUT_PULLUP);
#endif

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  Serial.println("|| ESP32 macro defined, setting up BLE server...");
  BLEMidiServer.begin("el_tuts");

  BLEMidiServer.setOnConnectCallback([]() {
    Serial.println("BLE Controller connected!");
    analogWrite(LED_PIN, 255);
  });

  BLEMidiServer.setOnDisconnectCallback([]() { Serial.println("BLE controller disconnected!"); });
#endif

  analogReadResolution(10);

  Serial.println("|| System ready <(':'<)\n");
}

const bool isConnected() {
#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  return BLEMidiServer.isConnected();
#else
  return true;
#endif
}

void loop() {
#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
  if (!digitalRead(RESET_PIN)) {
    SCB_AIRCR = 0x05FA0004;
  }
#endif

  if (isConnected()) {
    for (MidiSensor *SENSOR : SENSORS) {
      if (!SENSOR->isSwitchActive()) {
        continue;
      }

      SENSOR->run();
    }
  } else {
    Utils::blinkDisconnectedLedState(LED_PIN);
  }

  delayMicroseconds(100);
}
