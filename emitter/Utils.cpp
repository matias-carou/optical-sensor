#include "Utils.h"

#include <Arduino.h>

#include <map>

#include "Config.h"
#include "I2Cdev.h"
#include "types.h"

namespace Utils {
void printRuntimeOverrallValue(
    int& counter, int& timeBuffer, unsigned long& previousTime, unsigned long& currentTime, uint8_t CYCLES_AMOUNT) {
  if (counter % CYCLES_AMOUNT == 0 && counter != 0) {
    Serial.print("Average running time of ");
    Serial.print(CYCLES_AMOUNT);
    Serial.print(" cycles: ");
    Serial.print(timeBuffer);
    Serial.print("\t");
    Serial.print("Average running time of one cycle: ");
    const int diff = timeBuffer / counter;
    Serial.println(diff);
    timeBuffer = 0;
    counter = 0;
  }
  const int timeDiff = currentTime - previousTime;
  previousTime = currentTime;
  timeBuffer += timeDiff;
  counter++;
}
void checkForI2CDevices(TwoWire* wire) {
  byte error, address;
  int devicesFound = 0;

  Serial.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    wire->beginTransmission(address);
    error = wire->endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      devicesFound++;
    }
  }
}
void printMidiMessage(uint8_t& byte1, uint8_t& byte2, uint8_t& byte3) {
  Serial.print(F("MIDI message: "));
  Serial.print(byte1);
  Serial.print(F(" || "));
  Serial.print(F("Controller number: "));
  Serial.print(byte2);
  Serial.print(F(" || "));
  Serial.print(F("Value: "));
  Serial.println(byte3);
}
void blinkDisconnectedLedState(const LedPinsArray ledPins, unsigned long& currentTime, bool& ledToggleState) {
  if (millis() - currentTime >= 500) {
    for (const int ledPin : ledPins) {
      if (ledToggleState) {
        analogWrite(ledPin, 255);
      } else {
        analogWrite(ledPin, 0);
      }
    }

    ledToggleState = !ledToggleState;
    currentTime = millis();
  }
}

std::string getMicrocontrollerReadableValue() {
  std::map<int, std::string> data;

#ifdef MICROCONTROLLER_ESP32
  data.insert({ MICROCONTROLLER_ESP32, "ESP32" });
#endif
#ifdef MICROCONTROLLER_TEENSY
  data.insert({ MICROCONTROLLER_TEENSY, "Teensy" });
#endif
#ifdef MICROCONTROLLER_STM32
  data.insert({ MICROCONTROLLER_STM32, "STM32" });
#endif

#ifdef MICROCONTROLLER
  const auto it = data.find(MICROCONTROLLER);

  if (it != data.end()) {
    return it->second;
  }
#endif

  return "non-supported";
}
}  // namespace Utils
