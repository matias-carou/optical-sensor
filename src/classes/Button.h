#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

#include "DisplayManager.h"

const unsigned long LONG_PRESS_THRESHOLD = 500;  // For going back to root menu
extern DisplayManager &display;

struct Button {
  int pin;
  bool state;
  bool lastReading;
  unsigned long lastDebounceTime;
  unsigned long buttonPressTime = 0;
  bool longPressTriggered = false;
  bool shortPressHandled = false;  // New flag to track if we've handled the short press
  int debounceDelay = 75;
  bool pressHandled = false;

  Button(int p, int debounceDelay)
      : pin(p),
        state(HIGH),
        lastReading(HIGH),
        lastDebounceTime(0),
        buttonPressTime(0),
        longPressTriggered(false),
        shortPressHandled(false),
        debounceDelay(debounceDelay) {
    pinMode(pin, INPUT_PULLUP);
  }

  bool isDebounced() {
    bool reading = digitalRead(pin);

    if (reading != lastReading) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != state) {
        state = reading;
        // Only trigger on release (HIGH) if:
        // 1. No long press occurred (longPressTriggered is false)
        // 2. Press hasn't been handled yet (pressHandled is false)
        if (state == HIGH && !longPressTriggered && !pressHandled) {
          lastReading = reading;
          pressHandled = true;
          return true;
        }
      }
    }

    // Only reset pressHandled when a new press starts
    if (reading == LOW && state == HIGH) {  // New press detected
      pressHandled = false;
      longPressTriggered = false;  // Reset long press flag for new press
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
        pressHandled = true;  // Prevent short press from triggering
        return true;
      }
    } else {
      buttonPressTime = 0;
      // Don't reset longPressTriggered here, let it reset on new press
    }
    return false;
  }
};

#endif