#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#include <regex>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "I2Cdev.h"
#include "Utils.h"
#include "classes/Button.h"
#include "classes/DisplayManager.h"
#include "classes/MidiSensor.h"
#include "constants/MenuConfig.h"
#include "constants/animations/DisconnectedState.h"
#include "esp_heap_caps.h"
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

Encoder myEnc(CLK_PIN, DT_PIN);
Button encoderBtn(ENCODER_BUTTON_PIN);

JsonDocument oledConfigDoc;
JsonObject rootSensorsMenu;

JsonDocument parseJson(JsonDocument &docToRead) {
  const auto nestingLimit = DeserializationOption::NestingLimit(15);
  DeserializationError error = deserializeJson(oledConfigDoc, MENU_CONFIG, nestingLimit);

  if (error) {
    display.showText("Json Parse Failed");
    Serial.println(error.f_str());
    while (true);
  }

  if (!docToRead.is<JsonArray>()) {
    display.showText("Root JSON Not Iterable");
    while (true);
  }

  return docToRead;
}

void setup() {
  Serial.begin(9600);

  pinMode(encoderBtn.pin, INPUT);

  while (!Serial);
  // Serial.end();

  display.init();

  display.showText("Getting OLED Config...");

  parseJson(oledConfigDoc);

  display.showText("Parsed OLED Config...");
  Utils::printHeapInfo(2000);

  if (!oledConfigDoc.is<JsonArray>()) {
    display.showText("Root JSON Not Iterable");
    while (true);
  }

  rootSensorsMenu = oledConfigDoc[0]["submenu"].as<JsonObject>();

  if (rootSensorsMenu.isNull()) {
    display.showText("OLED Parse Failed");
    while (true);
  }

  display.setMenu(rootSensorsMenu);
  const JsonArray menuData = display.getMenu()["data"];

  // Just for debugging purposes
  // Utils::renderMenu(rootSensorsMenu, display);

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

struct SelectedOption {
  std::string id;
  std::string label;
  std::string value;
  int32_t selectedIndex;
};

const char *selectedOptionId = "";

SelectedOption selectedOption = { id : "", label : "", value : "", selectedIndex : 0 };

void goBack(const char *currentMenuId) {
  if (currentMenuId) {
    const std::string parsedId = std::string(currentMenuId);
    const std::regex subMenuPattern("\\.[^.]+$");
    std::string output = std::regex_replace(parsedId, subMenuPattern, "");
    display.showText(output.c_str());
    delay(1000);
  }
}

void runDisconnectedState() {
  menuInitialized = false;
  DisplayManager::displayAnimation(DISCONNECTED_FRAMES);
  blinkDisconnectedLedState(ledPins, currentTime, ledToggleState);
}

void initializeMenu(bool &menuInitialized) {
  display.setMenu(rootSensorsMenu);

  for (const int ledPin : ledPins) {
    analogWrite(ledPin, 255);
  }

  const JsonObject currentMenu = display.getMenu();

  Utils::validateMenu(currentMenu);

  const JsonArray menuData = currentMenu["data"];
  maxEncoderValue = static_cast<int>(menuData.size());
  oldPosition = -999;
  display.clear();
  menuInitialized = true;
}

/*
 * TODO: implement JSON indexing
 */
// const JsonDocument indexedDoc = parseSensorsMenu(oledConfigDoc);

void loop() {
  if (!BLEMidiServer.isConnected()) {
    return runDisconnectedState();
  }

  if (!menuInitialized) {
    initializeMenu(menuInitialized);
  }

  /*
  ** Get current menu data
  */
  const JsonObject currentMenu = display.getMenu();
  const JsonArray currentMenuData = currentMenu["data"].as<JsonArray>();

  /*
   * Menu logic if encoder turns
   */
  long rawPosition = myEnc.read();
  const bool isFullTurn = rawPosition % 4 == 0;
  const char *currentMenuLabel = currentMenu["label"];
  const char *currentMenuId = currentMenu["id"];

  if (isFullTurn) {
    newPosition = ((-rawPosition / 4) % maxEncoderValue + maxEncoderValue) % maxEncoderValue;

    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      const std::string label = currentMenuData[newPosition]["label"];
      auto value = currentMenuData[newPosition]["value"];

      if (currentMenuLabel) {
        std::string labelToPrint = "| " + std::string(currentMenuLabel) + " |";

        if (value && currentMenuLabel) {
          if (value.is<int>()) {
            value = std::to_string(value.as<int>());
          }

          std::string possibleDecoratedLabel = label;

          const std::string castedValue = value;

          if (std::string(currentMenuId) == selectedOption.id && castedValue == selectedOption.value &&
              newPosition == selectedOption.selectedIndex) {
            possibleDecoratedLabel = "* " + label + " *";
          }

          display.renderMultilineText({ labelToPrint.c_str(), possibleDecoratedLabel.c_str() });
        } else {
          display.renderMultilineText({ labelToPrint.c_str(), label.c_str() });
        }
      } else {
        display.showText(label.c_str());
      }
    }
  }

  if (encoderBtn.isLongPressed()) {
    goBack(currentMenuId);
    return;
  }

  if (encoderBtn.isDebounced()) {
    const JsonObject nestedSubMenu = currentMenuData[newPosition]["submenu"];

    if (nestedSubMenu) {
      JsonObject newSelectedMenu = nestedSubMenu;
      display.setMenu(newSelectedMenu);

      if (!newSelectedMenu["id"]) {
        display.showText("Failed to get menu ID");
        delay(500);
      }

      selectedOptionId = newSelectedMenu["id"];
      maxEncoderValue = static_cast<int>(newSelectedMenu["data"].size());
      newPosition = 0;
      oldPosition = -999;
    } else {
      const auto dataItem = currentMenuData[newPosition];

      if (dataItem) {
        const std::string menuLabel = dataItem["label"];
        auto value = dataItem["value"];

        if (value.is<int>()) {
          value = std::to_string(value.as<int>());
        }

        selectedOption = { id : selectedOptionId, label : menuLabel, value : value, selectedIndex : newPosition };

        const std::string selectedValue = "* " + menuLabel + " *";

        if (currentMenuLabel) {
          std::string labelToPrint = "| " + std::string(currentMenuLabel) + " |";
          display.renderMultilineText({ labelToPrint.c_str(), selectedValue.c_str() });
        } else {
          display.showText(selectedOption.label.c_str());
        }
      }
    }
  }

  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }

    SENSOR->run();
  }

  delayMicroseconds(500);
}