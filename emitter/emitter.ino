/**
 * arduino-cli compile --fqbn esp32:esp32:esp32c3 . && arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn esp32:esp32:esp32c3 . &&
 * arduino-cli monitor -p /dev/cu.usbmodem2101
 */

#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MidiSensor.h"
#include "Utils.h"

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
#  include <BLEMidi.h>
#endif

std::vector<MidiSensor *> SENSORS = {};

const std::array<int, 3> ledPins = { 1, 3, 0 };
unsigned long currentTime = 0;
bool ledToggleState = true;

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
const int RESET_PIN = 2;
#endif

void setup() {
  Serial.begin(9600);

  const std::string microControllerValue = Utils::getMicrocontrollerReadableValue();

  const std::string str = "|| Running code for microcontroller " + microControllerValue;
  Serial.println(str.c_str());

  SENSORS = MidiSensor::initializeSensors();

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
  pinMode(RESET_PIN, INPUT_PULLUP);
#endif

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  Serial.println("|| ESP32 macro defined, setting up BLE server...");
  BLEMidiServer.begin("el_tuts");

  BLEMidiServer.setOnConnectCallback([]() {
    Serial.println("BLE Controller connected!");

    for (const int ledPin : ledPins) {
      analogWrite(ledPin, 255);
    }
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
  // #if MICROCONTROLLER == MICROCONTROLLER_TEENSY
  //   if (!digitalRead(RESET_PIN)) {
  //     SCB_AIRCR = 0x05FA0004;
  //   }
  // #endif

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  if (isConnected()) {
#endif
    for (MidiSensor *SENSOR : SENSORS) {
      if (!SENSOR->isSwitchActive()) {
        continue;
      }

      SENSOR->run();
    }
#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  } else {
    Utils::blinkDisconnectedLedState(ledPins, currentTime, ledToggleState);
  }
#endif

  delayMicroseconds(500);
}
