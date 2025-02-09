#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

using LedPinsArray = std::array<int, 2>;
using TDependency = const std::vector<std::string>;

struct SelectedOption {
  std::string id;
  std::string label;
  std::string value;
  std::vector<std::string> dependencies;
  int32_t selectedIndex;
};
