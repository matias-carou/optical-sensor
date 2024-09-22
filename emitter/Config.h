#pragma once

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

extern const char *CONFIG;
