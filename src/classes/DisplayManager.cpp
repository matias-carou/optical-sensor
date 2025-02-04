#include "DisplayManager.h"

#include <ArduinoJson.h>

#include <vector>

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
void DisplayManager::renderMultilineText(std::initializer_list<const char*> lines, const bool clearDisplay) {
  if (clearDisplay) {
    display.clearDisplay();
  }

  const int SPACING = 2;

  int totalTextHeight = 0;
  std::vector<uint16_t> lineHeights;
  lineHeights.reserve(lines.size());

  for (const char* line : lines) {
    int16_t dummyX, dummyY;
    uint16_t width, height;
    display.getTextBounds(line, 0, 0, &dummyX, &dummyY, &width, &height);
    lineHeights.push_back(height);
    totalTextHeight += height;
  }

  totalTextHeight += SPACING * (lines.size() - 1);

  int yPos = (SCREEN_HEIGHT - totalTextHeight) / 2;

  struct lineObject {
    const char* line;
    int textSize = 1;

    lineObject(const char* l, int ts = 1) : line(l), textSize(ts) {
    }
  };

  std::vector<lineObject> arr;

  int textSize = 1;

  for (const char* line : lines) {
    arr.push_back(lineObject(line));
  }

  auto heightIt = lineHeights.begin();
  for (const lineObject lineData : arr) {
    display.setTextSize(lineData.textSize);
    int16_t x, y;
    uint16_t width, height;
    display.getTextBounds(lineData.line, 0, 0, &x, &y, &width, &height);

    int centerX = (SCREEN_WIDTH - width) / 2;
    display.setCursor(centerX, yPos);
    display.println(lineData.line);

    yPos += height + SPACING;
    ++heightIt;
  }

  display.display();
}
