#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Encoder.h>

#include <regex>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "I2Cdev.h"
#include "Utils.h"
#include "classes/ActionService.h"
#include "classes/Button.h"
#include "classes/DisplayManager.h"
#include "classes/MidiSensor.h"
#include "constants/MenuConfig.h"
#include "constants/animations/Burger.h"
#include "constants/animations/DisconnectedState.h"
#include "esp_heap_caps.h"
#include "types.h"

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
#  include <BLEMidi.h>
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
    display.showText("Failed to Parse Menu");
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

  parseJson();

  Utils::validateMenu(rootMenu);

  // Utils::printHeapInfo(500);

  castedRoot = rootMenu.as<JsonObject>();
  display.setMenu(castedRoot);

  // Just for debugging purposes
  // Utils::renderMenu(rootSensorsMenu, display);

  SENSORS = MidiSensor::initializeSensors();

#if MICROCONTROLLER == MICROCONTROLLER_ESP32
  Serial.println("|| ESP32 macro defined, setting up BLE server...");
  BLEMidiServer.begin("el_tuts");

  BLEMidiServer.setOnConnectCallback([]() {
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
}

long oldPosition = -999;

unsigned long prevDisconnectedTime = 0;

bool menuInitialized = false;
int maxEncoderValue = -999;
int32_t newPosition = 0;

SelectedOption selectedOption = { id : "", label : "", value : "", dependencies : {}, selectedIndex : 0 };
const char *selectedOptionId = "";

void handleButtonLongPress() {
  const JsonObject currentMenu = display.getMenu();
  const char *currentMenuId = currentMenu["id"].as<const char *>();

  if (!currentMenuId) {
    display.showText("Item ID Not Found");
    return;
  }

  display.setMenu(castedRoot);
  display.showText("Back to Root...");
  delay(250);
  const std::string previousMenuLabel = castedRoot["label"];
  const std::string label = castedRoot["data"][0]["label"];
  display.renderMultilineText({ previousMenuLabel.c_str() }, { label.c_str() });
  maxEncoderValue = static_cast<int>(castedRoot["data"].size());
}

void handleButtonPress() {
  const JsonObject currentMenu = display.getMenu();
  const JsonArray currentMenuData = currentMenu["data"];

  if (currentMenuData.size() > 0) {
    const JsonObject nestedSubMenu = currentMenuData[newPosition];
    const bool hasMoreData = nestedSubMenu["data"] && !!nestedSubMenu["data"].size();

    if (hasMoreData) {
      // display.setPreviousMenu(currentMenu); // TODO: figure out how to actually handle this
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

        std::vector<std::string> dependencies;
        const JsonArray jsonDependencies = currentMenu["dependencies"] ? currentMenu["dependencies"] : JsonArray();
        dependencies.reserve(jsonDependencies.size());
        for (const auto &value : jsonDependencies) {
          dependencies.push_back(value.as<const char *>());
        }

        if (dependencies.empty()) {
          display.showText("No dependencies found");
          delay(500);
        }

        selectedOption = {
          id : selectedOptionId,
          label : menuLabel,
          value : value,
          dependencies : dependencies,
          selectedIndex : newPosition,
        };

        ActionService::getInstance().dispatchAction(SENSORS, selectedOption);

        const std::string selectedValue = "* " + menuLabel + " *";
        const std::string currentLabel = display.getMenu()["label"];

        if (!currentLabel.empty()) {
          display.renderMultilineText({ currentLabel.c_str() }, { selectedValue.c_str() });
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
        std::string labelToPrint = std::string(currentMenuLabel);

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

          const std::string rootReplace = std::regex_replace(labelToPrint, std::regex("(root\\.)"), "");
          const std::string breadcrumbs = std::regex_replace(rootReplace, std::regex("(\\.)"), " > ");
          display.renderMultilineText({ breadcrumbs.c_str() }, { possibleDecoratedLabel.c_str() });
        } else {
          display.renderMultilineText({ labelToPrint.c_str() }, { label.c_str() });
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

// void runSensors() {
//   for (MidiSensor *SENSOR : SENSORS) {
//     if (!SENSOR->isSwitchActive()) {
//       continue;
//     }

//     SENSOR->run();
//   }
// }

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

void runEncoderHandler() {
  const long rawPosition = myEnc.read();
  const bool isFullTurn = rawPosition % 4 == 0;

  if (isFullTurn) {
    handleButtonMovement(rawPosition);
  }

  if (encoderBtn.isLongPressed()) {
    handleButtonLongPress();
  } else if (encoderBtn.isDebounced()) {
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
  ActionService::getInstance().runSensors(SENSORS);

  delayMicroseconds(100);
}