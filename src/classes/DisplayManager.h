#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

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

  void init();
  void clear();
  void showText(const char* text, const bool clearDisplay = true);
  void setTextSize(const int testSize);

 private:
  Adafruit_SSD1306 display;

  DisplayManager();  // Private constructor
  ~DisplayManager() = default;
  DisplayManager(const DisplayManager&) = delete;
  DisplayManager& operator=(const DisplayManager&) = delete;
};

#endif
