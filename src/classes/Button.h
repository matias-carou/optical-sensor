#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

#include "DisplayManager.h"

#define DEBOUNCE_DELAY 100

const unsigned long LONG_PRESS_THRESHOLD = 1000;
bool buttonPressed = false;
extern DisplayManager &display;

struct Button {
  int pin;
  bool state;
  bool lastReading;
  unsigned long lastDebounceTime;
  unsigned long buttonPressTime = 0;
  bool longPressTriggered = false;

  Button(int p) : pin(p), state(HIGH), lastReading(HIGH), lastDebounceTime(0), buttonPressTime(0), longPressTriggered(false) {
    pinMode(pin, INPUT_PULLUP);
  }

  bool isDebounced() {
    bool reading = digitalRead(pin);

    if (reading != lastReading) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
      if (reading != state) {
        state = reading;
        if (state == LOW) {
          lastReading = reading;
          return true;
        }
      }
    }

    lastReading = reading;
    return false;
  }

  bool isLongPressed() {
    bool reading = digitalRead(pin);

    if (reading == LOW) {
      if (buttonPressTime == 0) {
        buttonPressTime = millis();
      }
      if (!longPressTriggered && (millis() - buttonPressTime >= LONG_PRESS_THRESHOLD)) {
        longPressTriggered = true;
        return true;
      }
    } else {
      buttonPressTime = 0;
      longPressTriggered = false;
    }
    return false;
  }
};

#endif
