#pragma once
#include <Arduino.h>

#include <string>

/**
 * Encoder
 */
#define ENCODER_CLK_PIN 21
#define ENCODER_DT_PIN 20
#define ENCODER_BUTTON_PIN 4
#define DEBOUNCE_DELAY 75

/**
 * Microcontroller
 */
#define DEBUG 0
#define MICROCONTROLLER_ESP32 1
#define MICROCONTROLLER_TEENSY 2

/**
 * Define the microcontroller to be used in the project
 * Teensy: Serial Communication (Deprecated)
 * ESP32: BLE Communication
 */
#define MICROCONTROLLER MICROCONTROLLER_ESP32
// #define MICROCONTROLLER MICROCONTROLLER_TEENSY

#include "constants/EspConfig.h"
#include "constants/TeensyConfig.h"

inline const char *getConfig() {
#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  return ESP32_CONFIG;
#elif MICROCONTROLLER == MICROCONTROLLER_TEENSY
  return TEENSY_CONFIG;
#else
  return "";
#endif
}