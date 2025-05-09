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

void DisplayManager::renderBitmap(const unsigned char frame[]) {
  display.clearDisplay();
  display.drawBitmap(32, 0, frame, 64, 64, WHITE);  // TODO: make this dynamic
  display.display();
}

void DisplayManager::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  display.fillRect(x, y, w, h, color);
}

// TODO: Actually implement or remove this
void DisplayManager::setPreviousMenu(JsonObject menu) {
  previousMenu = menu;
}

JsonObject DisplayManager::getPreviousMenu() {
  return previousMenu;
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

void DisplayManager::renderMultilineText(std::vector<const char*> filterLines,
                                         std::vector<const char*> optionLines,
                                         const bool clearDisplay) {
  display.setTextSize(1);

  if (clearDisplay) {
    display.clearDisplay();
  }

  // Render the Filter section
  int filterTextHeight = 0;
  std::vector<uint16_t> filterLineHeights;
  filterLineHeights.reserve(filterLines.size());

  for (const char* line : filterLines) {
    int16_t dummyX, dummyY;
    uint16_t width, height;
    display.getTextBounds(line, 0, 0, &dummyX, &dummyY, &width, &height);
    filterLineHeights.push_back(height);
    filterTextHeight += height;
  }

  const int SPACING = 2;
  const int FILTER_HEIGHT = SCREEN_HEIGHT / 6;  // Reserve 1/6 of the screen for the Filter section
  const int OPTION_HEIGHT = SCREEN_HEIGHT - FILTER_HEIGHT;
  filterTextHeight += SPACING * (filterLines.size() - 1);
  int filterYPos = (FILTER_HEIGHT - filterTextHeight) / 2;

  // Check if there are filter lines to render
  if (!filterLines.size()) {
    // If no filter lines, set filterYPos to 0
    filterYPos = 0;
  }

  for (const char* line : filterLines) {
    int16_t x, y;
    uint16_t width, height;
    display.getTextBounds(line, 0, 0, &x, &y, &width, &height);

    int centerX = (SCREEN_WIDTH - width) / 2;
    display.setCursor(centerX, filterYPos);
    display.println(line);

    filterYPos += height + SPACING;
  }

  const int LINE_SPACING = 4;
  int lineYPos = FILTER_HEIGHT + LINE_SPACING;
  int lineWidth = SCREEN_WIDTH - 16;                // Set desired line width (adjust as needed)
  int lineStartX = (SCREEN_WIDTH - lineWidth) / 2;  // Center the line horizontally
  int lineEndX = lineStartX + lineWidth;

  int optionTextHeight = 0;
  // Draw the separator line only if there are filter lines
  if (!filterLines.size()) {
    lineYPos = (SCREEN_HEIGHT - (optionTextHeight + LINE_SPACING)) / 2;  // Center line if no filter
  } else {
    display.drawLine(lineStartX, lineYPos, lineEndX, lineYPos, SSD1306_WHITE);
  }

  // Render the Option section
  std::vector<uint16_t> optionLineHeights;
  optionLineHeights.reserve(optionLines.size());

  for (const char* line : optionLines) {
    int16_t dummyX, dummyY;
    uint16_t width, height;
    display.getTextBounds(line, 0, 0, &dummyX, &dummyY, &width, &height);
    optionLineHeights.push_back(height);
    optionTextHeight += height;
  }

  optionTextHeight += SPACING * (optionLines.size() - 1);
  int optionYPos;

  // Center options if there are no filter lines
  if (filterLines.size() == 0) {
    optionYPos = (SCREEN_HEIGHT - optionTextHeight) / 2;  // Center options vertically
  } else {
    optionYPos = FILTER_HEIGHT + (OPTION_HEIGHT - optionTextHeight) / 2;  // Center options below filter
  }

  for (const char* line : optionLines) {
    int16_t x, y;
    uint16_t width, height;
    display.getTextBounds(line, 0, 0, &x, &y, &width, &height);

    int centerX = (SCREEN_WIDTH - width) / 2;
    display.setCursor(centerX, optionYPos);
    display.println(line);

    optionYPos += height + SPACING;
  }

  display.display();
}