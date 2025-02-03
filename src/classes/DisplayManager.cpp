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

void DisplayManager::setMenu(JsonObject submenu) {
  menu = submenu;
}

JsonObject DisplayManager::getMenu() {
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

// TODO: Iterate
void DisplayManager::renderMultilineText(const char* line1, const char* line2, const bool clearDisplay) {
  if (clearDisplay) {
    display.clearDisplay();
  }

  int16_t x1, y1, x2, y2;
  uint16_t width1, height1, width2, height2;
  display.getTextBounds(line1, 0, 0, &x1, &y1, &width1, &height1);
  display.getTextBounds(line2, 0, 0, &x2, &y2, &width2, &height2);

  const int spacing = 2;

  const int totalTextHeight = height1 + spacing + height2;

  const int startY = (SCREEN_HEIGHT - totalTextHeight) / 2;

  const int centerX1 = (SCREEN_WIDTH - width1) / 2;
  const int centerX2 = (SCREEN_WIDTH - width2) / 2;

  display.setCursor(centerX1, startY);
  display.println(line1);

  display.setCursor(centerX2, startY + height1 + spacing);
  display.println(line2);

  display.display();
}
