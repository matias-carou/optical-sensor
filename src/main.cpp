#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "I2Cdev.h"
#include "Utils.h"
#include "classes/DisplayManager.h"
#include "classes/MidiSensor.h"
#include "constants/MenuConfig.h"
#include "constants/animations/DisconnectedState.h"
#include "types.h"

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
#  include <BLEMidi.h>
#endif
#include <Encoder.h>

std::vector<MidiSensor *> SENSORS = {};

const LedPinsArray ledPins = { 2, 3 };
unsigned long currentTime = 0;
bool ledToggleState = true;

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
const int RESET_PIN = 2;
#endif

using namespace std;
using namespace Utils;

DisplayManager &display = DisplayManager::getInstance();

#define CLK_PIN 21            // Connect to Pin 1 (CLK)
#define DT_PIN 20             // Connect to Pin 2 (DT)
#define ENCODER_BUTTON_PIN 4  // Connect to Pin 4 (SW)
#define BUTTON_DEBOUNCE_DELAY 2000

Encoder myEnc(CLK_PIN, DT_PIN);

bool lastButtonState = false;
unsigned long lastDebounceTime = 0;

JsonDocument oledConfig = parseMenuConfig();
JsonArray sensorsMenu = oledConfig[0]["submenu"].as<JsonArray>();

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_BUTTON_PIN, INPUT);

  while (!Serial);
  // Serial.end();

  display.init();

  // TODO: Make This Recursive
  if (sensorsMenu.isNull()) {
    display.showText("OLED Parse Failed");
    while (true);
  } else {
    display.setMenu(sensorsMenu);

    for (JsonObject menuItem : display.getMenu()) {
      const char *label = menuItem["label"].as<const char *>();

      if (label) {
        display.showText(label);
        delay(250);

        if (menuItem["submenu"]) {
          display.setMenu(menuItem["submenu"]);

          for (JsonObject subMenuItem : display.getMenu()) {
            display.showText(subMenuItem["label"].as<const char *>());
            delay(25);

            if (subMenuItem["submenu"]) {
              display.setMenu(subMenuItem["submenu"]);

              for (JsonObject nestedMenuItem : display.getMenu()) {
                display.showText(nestedMenuItem["label"].as<const char *>());
                delay(25);
              }
            }
          }
        }
      }
    }
  }

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
    display.showText("| Connected |");
    display.setTextSize(1);

    for (const int ledPin : ledPins) {
      analogWrite(ledPin, 255);
    }
  });

  BLEMidiServer.setOnDisconnectCallback([]() {
    display.setTextSize(2);
    Serial.println("BLE controller disconnected!");
  });
#endif

  analogReadResolution(10);

  Serial.println("|| (>':')> System ready <(':'<) ||\n");
}

long oldPosition = -999;

// TODO: implement, either way encoder click is pretty stable
bool debounceButton(int pin, unsigned long &previousTime) {
  return !digitalRead(pin);
}

unsigned long prevDisconnectedTime = 0;

bool menuInitialized = false;
int maxEncoderValue = -999;
int32_t newPosition = 0;

void loop() {
  if (!BLEMidiServer.isConnected() && millis() - prevDisconnectedTime >= 500) {
    menuInitialized = false;
    DisplayManager::displayAnimation(DISCONNECTED_FRAMES);
    return blinkDisconnectedLedState(ledPins, currentTime, ledToggleState);
    prevDisconnectedTime = millis();
  } else {
    if (!menuInitialized) {
      menuInitialized = true;
      display.setMenu(sensorsMenu);
      maxEncoderValue = static_cast<int>(display.getMenu().size());
      oldPosition = -999;
    }

    const bool isButtonPressed = debounceButton(ENCODER_BUTTON_PIN, lastDebounceTime);

    long rawPosition = myEnc.read();
    const bool isFullTurn = rawPosition % 4 == 0;
    const JsonArray currentMenu = display.getMenu();

    if (isFullTurn) {
      // int32_t newPosition = ((-rawPosition / 4) % maxEncoderValue + maxEncoderValue) % maxEncoderValue;
      newPosition = ((-rawPosition / 4) % maxEncoderValue + maxEncoderValue) % maxEncoderValue;

      if (newPosition != oldPosition) {
        oldPosition = newPosition;
        const auto label = currentMenu[newPosition]["label"];

        if (label) {
          display.showText(label.as<const char *>());
        } else {
          display.showText("Label Not Defined");
        }
      }
    }

    if (isButtonPressed) {
      delay(750);  // TODO: implement debounce
      const JsonArray nestedSubMenu = currentMenu[newPosition]["submenu"];

      if (nestedSubMenu) {
        JsonArray newSelectedMenu = nestedSubMenu;
        display.setMenu(newSelectedMenu);
        maxEncoderValue = static_cast<int>(newSelectedMenu.size());
        newPosition = 0;
        oldPosition = newPosition;
      } else {
        display.showText("Submenu N/A");
      }
    }

    for (MidiSensor *SENSOR : SENSORS) {
      if (!SENSOR->isSwitchActive()) {
        continue;
      }

      SENSOR->run();
    }
  }

  delayMicroseconds(500);
}