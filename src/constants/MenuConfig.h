#pragma once
#include <ArduinoJson.h>

#include <string>
#include <vector>

extern const char *MENU_CONFIG;

// Menu Structures
struct Step {
  std::string path;
  std::string label;
  int value = 0;
  int minValue = 0;
  int maxValue = 0;
};

struct MenuItem {
  std::string path;
  std::string label;
  std::vector<Step> steps;
  std::vector<MenuItem> submenu;
};

// Function to parse JSON & return menu structure
JsonDocument parseMenuConfig();
