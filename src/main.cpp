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
#define BUTTON_DEBOUNCE_DELAY 2000

Encoder myEnc(CLK_PIN, DT_PIN);

bool lastButtonState = false;
unsigned long lastDebounceTime = 0;

void printHeapInfo() {
  Serial.printf("Total Heap: %d bytes\n", ESP.getHeapSize());
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Used Heap: %d bytes\n", ESP.getHeapSize() - ESP.getFreeHeap());
  Serial.printf("Largest Free Block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.println("-----------------------------");
}

JsonDocument oledConfigDoc;
JsonObject rootSensorsMenu;

JsonDocument parseJson(JsonDocument &docToRead) {
  const auto nestingLimit = DeserializationOption::NestingLimit(15);
  DeserializationError error = deserializeJson(oledConfigDoc, MENU_CONFIG, nestingLimit);

  if (error) {
    Serial.print("❌ JSON Parsing Failed: ");
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

  pinMode(ENCODER_BUTTON_PIN, INPUT);

  while (!Serial);
  // Serial.end();

  display.init();

  display.showText("Getting OLED Config...");

  parseJson(oledConfigDoc);

  display.showText("Parsed OLED Config...");

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
  display.showText("Oled JSON Parse OK");
  delay(500);

  // Just for debugging purposes
  Utils::renderMenu(rootSensorsMenu, display);

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
    // display.setMenu(rootSensorsMenu);

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
};

const char *selectedOptionId = "";
bool showMultiline = false;

void loop() {
  if (!BLEMidiServer.isConnected()) {
    menuInitialized = false;
    DisplayManager::displayAnimation(DISCONNECTED_FRAMES);
    return blinkDisconnectedLedState(ledPins, currentTime, ledToggleState);
  }

  if (!menuInitialized) {
    display.showText("Setting up Main Menu");
    display.setMenu(rootSensorsMenu);

    delay(500);

    const JsonObject currentMenu = display.getMenu();
    printHeapInfo();

    if (currentMenu.isNull()) {
      display.showText("Invalid JSON Object");
      while (true);
    }

    if (!currentMenu.containsKey("data")) {
      display.showText("Failed to get menu data");
    }

    const JsonArray menuData = currentMenu["data"];
    printHeapInfo();
    delay(500);
    maxEncoderValue = static_cast<int>(menuData.size());
    oldPosition = -999;
    display.clear();
    menuInitialized = true;
  }

  const bool isButtonPressed = debounceButton(ENCODER_BUTTON_PIN, lastDebounceTime);

  /*
  ** Get encoder data
  */
  long rawPosition = myEnc.read();
  const bool isFullTurn = rawPosition % 4 == 0;
  const JsonObject currentMenu = display.getMenu();

  const JsonArray currentMenuData = currentMenu["data"].as<JsonArray>();
  const char *currentMenuLabel = currentMenu["label"];

  /*
   * Menu logic if encoder turns
   */
  if (isFullTurn) {
    newPosition = ((-rawPosition / 4) % maxEncoderValue + maxEncoderValue) % maxEncoderValue;

    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      const auto label = currentMenuData[newPosition]["label"];
      auto value = currentMenuData[newPosition]["value"];

      if (value && currentMenuLabel) {
        if (value.is<int>()) {
          value = std::to_string(value.as<int>());
        }
        display.renderMultilineText(currentMenuLabel, value);
      } else {
        display.showText(label.as<const char *>());
      }
    }
  }

  if (isButtonPressed) {
    delay(750);  // TODO: implement debounce
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
      showMultiline = false;
    } else {
      // showMultiline = true;
      const auto dataItem = currentMenuData[newPosition];

      if (dataItem) {
        const auto menuLabel = dataItem["label"];
        auto value = dataItem["value"];

        if (value.is<int>()) {
          value = std::to_string(value.as<int>());
        }

        // const std::string printData = "* " + std::string(menuLabel.as<const char *>()) + " *";
        // display.showText(printData.c_str());

        const SelectedOption selectedOption = { id : selectedOptionId, label : menuLabel, value : value };
        const std::string labelString = "label: " + selectedOption.label;

        display.showText("Value Selected");
        delay(500);
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