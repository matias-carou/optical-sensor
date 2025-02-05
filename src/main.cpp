#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Encoder.h>

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
#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
const int RESET_PIN = 2;
#endif

std::vector<MidiSensor *> SENSORS = {};

const LedPinsArray ledPins = { 2, 3 };
unsigned long currentTime = 0;
bool ledToggleState = true;

using namespace std;
using namespace Utils;

DisplayManager &display = DisplayManager::getInstance();

Encoder myEnc(ENCODER_CLK_PIN, ENCODER_DT_PIN);
Button encoderBtn(ENCODER_BUTTON_PIN, DEBOUNCE_DELAY);

JsonDocument rootMenu;
JsonObject rootSensorsMenu;
JsonObject castedRoot;

JsonDocument parseJson() {
  const auto nestingLimit = DeserializationOption::NestingLimit(30);
  DeserializationError error = deserializeJson(rootMenu, MENU_CONFIG, nestingLimit);

  if (error) {
    display.showText("Json Parse Failed");
    Serial.println(error.f_str());
    while (true);
  }

  return rootMenu;
}

void setup() {
  Serial.begin(9600);

  pinMode(encoderBtn.pin, INPUT);

  while (!Serial);
  // Serial.end();

  display.init();

  display.showText("Getting OLED Config...");

  parseJson();

  display.showText("Parsed OLED Config...");

  Utils::validateMenu(rootMenu);

  Utils::printHeapInfo(500);

  castedRoot = rootMenu.as<JsonObject>();
  display.setMenu(castedRoot);

  const std::string label = display.getMenu()["label"];
  const std::string id = display.getMenu()["id"];

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

void handleButtonLongPress() {
  const JsonObject currentMenu = display.getMenu();
  const char *currentMenuId = currentMenu["id"].as<const char *>();

  if (!currentMenuId) {
    display.showText("Item ID Not Found");
    return;
  }

  const std::string parsedId = std::string(currentMenuId);
  const std::regex subMenuPattern("\\.[^.]+$");
  std::string output = std::regex_replace(parsedId, subMenuPattern, "");
  display.showText(output.c_str());
  delay(1000);
}

void handleButtonPress() {
  const JsonArray currentMenuData = display.getMenu()["data"];

  if (currentMenuData.size() > 0) {
    const JsonObject nestedSubMenu = currentMenuData[newPosition];
    const JsonArray hasMoreData = nestedSubMenu["data"];

    if (hasMoreData) {
      JsonObject newSelectedMenu = nestedSubMenu;
      display.setMenu(newSelectedMenu);

      if (newSelectedMenu["id"]) {
        selectedOptionId = newSelectedMenu["id"];
      } else {
        display.showText("Failed to get ID");
      }

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

        const std::string currentLabel = display.getMenu()["label"];

        if (!currentLabel.empty()) {
          std::string labelToPrint = "| " + currentLabel + " |";
          display.renderMultilineText({ labelToPrint.c_str(), selectedValue.c_str() });
        } else {
          display.showText(selectedOption.label.c_str());
        }
      }
    }
  }
}

void handleButtonMovement(const long rawPosition) {
  const JsonObject currentMenu = display.getMenu();
  const JsonArray currentMenuData = currentMenu["data"];

  if (currentMenuData.size() > 0) {
    const char *currentMenuLabel = currentMenu["label"];
    const char *currentMenuId = currentMenu["id"];
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
}

void runDisconnectedState(bool &menuInitialized) {
  menuInitialized = false;
  DisplayManager::displayAnimation(DISCONNECTED_FRAMES);
  blinkDisconnectedLedState(ledPins, currentTime, ledToggleState);
}

void runSensors() {
  for (MidiSensor *SENSOR : SENSORS) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }

    SENSOR->run();
  }
}

void initializeMenu(bool &menuInitialized) {
  display.setMenu(castedRoot);

  for (const int ledPin : ledPins) {
    analogWrite(ledPin, 255);
  }

  maxEncoderValue = static_cast<int>(display.getMenu()["data"].size());
  oldPosition = -999;
  display.clear();
  menuInitialized = true;
}

/*
 * TODO: implement JSON indexing
 */
// const JsonDocument indexedDoc = parseSensorsMenu(oledConfigDoc);

void runEncoderHandler() {
  const long rawPosition = myEnc.read();
  const bool isFullTurn = rawPosition % 4 == 0;

  if (isFullTurn) {
    handleButtonMovement(rawPosition);
  }

  if (encoderBtn.isLongPressed()) {
    handleButtonLongPress();
  }

  if (encoderBtn.isDebounced()) {
    handleButtonPress();
  }
}

void loop() {
  /*
   ** Run Disconnected (Advertising) State
   */
  if (!BLEMidiServer.isConnected()) {
    return runDisconnectedState(menuInitialized);
  }

  if (!menuInitialized) {
    initializeMenu(menuInitialized);
  }

  /*
  ** Everything related to the menu + encoder logic
  */
  runEncoderHandler();

  /*
  ** Run all sensors
  */
  runSensors();

  delayMicroseconds(500);
}