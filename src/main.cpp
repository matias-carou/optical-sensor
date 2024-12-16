#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "I2Cdev.h"
#include "Utils.h"
#include "classes/MidiSensor.h"
#include "types.h"

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
#  include <BLEMidi.h>
#endif

std::vector<MidiSensor *> SENSORS = {};

const LedPinsArray ledPins = { 2, 3 };
unsigned long currentTime = 0;
bool ledToggleState = true;

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
const int RESET_PIN = 2;
#endif

using namespace std;
using namespace Utils;

void setup() {
  Serial.begin(9600);

  while (!Serial);

  const string microControllerValue = Utils::getMicrocontrollerReadableValue();

  const string str = "|| Running code for microcontroller " + microControllerValue;
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

  Serial.println("|| (>':')> System ready <(':'<) ||\n");
}

/**
 * Render either Teensy or ESP32 config
 */
void loop() {
#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
  if (!digitalRead(RESET_PIN)) {
    SCB_AIRCR = 0x05FA0004;
  }

  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }

    SENSOR->run();
  }
#endif
#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  if (!BLEMidiServer.isConnected()) {
    return blinkDisconnectedLedState(ledPins, currentTime, ledToggleState);
  }

  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }

    SENSOR->run();
  }
#endif
  delayMicroseconds(500);
}