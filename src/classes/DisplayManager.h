#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "constants/animations/DisconnectedState.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class DisplayManager {
 public:
  static DisplayManager& getInstance() {
    static DisplayManager instance;
    return instance;
  }

  template <size_t N>
  static void displayAnimation(const AnimationFrame (&frames)[N]) {
    for (const auto& frameData : frames) {
      const auto& frame = frameData.frame;
      const auto& delayAfterFrame = frameData.delayAfterFrame;

      getInstance().display.clearDisplay();
      getInstance().display.drawBitmap(0, 0, frame, 128, 64, WHITE);
      getInstance().display.display();

      if (delayAfterFrame) {
        // TODO: implement non blocking delay
        delay(delayAfterFrame);
      }
    }
  }

  void init();
  void clear();
  void showText(const char* text, const bool clearDisplay = true);
  void setTextSize(const int testSize);
  void drawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void renderBitmap(const unsigned char frame[]);
  void setMenu(JsonObject submenu);
  void renderMultilineText(std::initializer_list<const char*> filterLines,
                           std::initializer_list<const char*> optionLines,
                           const bool clearDisplay = true);
  JsonObject getMenu();

 private:
  Adafruit_SSD1306 display;
  JsonObject menu;

  DisplayManager();  // Private constructor
  ~DisplayManager() = default;
  DisplayManager(const DisplayManager&) = delete;
  DisplayManager& operator=(const DisplayManager&) = delete;
};

#endif
