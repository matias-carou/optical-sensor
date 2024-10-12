#pragma once
#include <Arduino.h>

#include <string>

#define DEBUG 0
#define MICROCONTROLLER_ESP32 1
#define MICROCONTROLLER_TEENSY 2

/**
 * Define the microcontroller to be used in the project
 * Teensy: Serial Communication
 * ESP32: BLE Communication
 */
#define MICROCONTROLLER MICROCONTROLLER_ESP32
// #define MICROCONTROLLER MICROCONTROLLER_TEENSY

#include "EspConfig.h"
#include "TeensyConfig.h"

inline const char *getConfig() {
#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  return ESP32_CONFIG;
#elif MICROCONTROLLER == MICROCONTROLLER_TEENSY
  return TEENSY_CONFIG;
#else
  return "";
#endif
}