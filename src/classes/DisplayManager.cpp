#include "DisplayManager.h"

#include <ArduinoJson.h>

DisplayManager::DisplayManager() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
}

void DisplayManager::init() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.display();
}

void DisplayManager::setTextSize(const int testSize) {
  display.setTextSize(1);
}

void DisplayManager::drawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color) {
  display.clearDisplay();
  display.drawBitmap(x, y, bitmap, w, h, color);
  display.display();
}

void DisplayManager::setMenu(JsonArray submenu) {
  menu = submenu;
}

JsonArray DisplayManager::getMenu() {
  return menu;
}

void DisplayManager::clear() {
  display.clearDisplay();
  display.display();
}

void DisplayManager::showText(const char* text, const bool clearDisplay) {
  if (clearDisplay) {
    display.clearDisplay();
  }

  int16_t x, y;
  uint16_t width, height;
  display.getTextBounds(text, 0, 0, &x, &y, &width, &height);

  const int centerX = (SCREEN_WIDTH - width) / 2;
  const int centerY = (SCREEN_HEIGHT - height) / 2;

  display.setCursor(centerX, centerY);

  if (clearDisplay) {
    display.clearDisplay();
  }

  display.println(text);
  display.display();
}
