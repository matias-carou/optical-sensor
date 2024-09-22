#pragma once
#include <Arduino.h>

#include <map>
#include <sstream>
#include <vector>

#include "I2Cdev.h"
#include "types.h"

namespace Utils {
void printRuntimeOverrallValue(
    int& counter, int& timeBuffer, unsigned long& previousTime, unsigned long& currentTime, uint8_t CYCLES_AMOUNT = 20);
void checkForI2CDevices(TwoWire* wire);
void printMidiMessage(uint8_t& byte1, uint8_t& byte2, uint8_t& byte3);
void blinkDisconnectedLedState(const LedPinsArray ledPins, unsigned long& currentTime, bool& ledToggleState);
std::string getMicrocontrollerReadableValue();
}  // namespace Utils