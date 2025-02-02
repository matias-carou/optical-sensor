#include "MenuConfig.h"

#include <ArduinoJson.h>

const char *MENU_CONFIG = R"(
[
  {
    "label": "Sensors",
    "submenu": [
      {
        "label": "TOF",
        "submenu": [
          {
            "label": "Floor",
            "submenu": [
              {
                "label": "25",
                "value": 25
              },
              {
                "label": "50",
                "value": 50
              },
              {
                "label": "100",
                "value": 100
              },
              {
                "label": "150",
                "value": 150
              }
            ],
            "minValue": 0,
            "maxValue": 150
          },
          {
            "label": "Ceil",
            "submenu": [
              {
                "label": "250",
                "value": 250
              },
              {
                "label": "300",
                "value": 300
              },
              {
                "label": "350",
                "value": 350
              },
              {
                "label": "400",
                "value": 400
              }
            ],
            "minValue": 0,
            "maxValue": 400
          }
        ]
      },
      {
        "label": "IMU",
        "submenu": [
          {
            "label": "Floor",
            "submenu": [
              {
                "label": "50",
                "value": 50
              },
              {
                "label": "150",
                "value": 150
              },
              {
                "label": "200",
                "value": 200
              },
              {
                "label": "250",
                "value": 250
              }
            ],
            "minValue": 50,
            "maxValue": 250
          },
          {
            "label": "Ceil",
            "submenu": [
              {
                "label": "10000",
                "value": 10000
              },
              {
                "label": "12500",
                "value": 12500
              },
              {
                "label": "15000",
                "value": 15000
              },
              {
                "label": "16000",
                "value": 16000
              }
            ],
            "minValue": 10000,
            "maxValue": 16000
          }
        ]
      }
    ]
  }
]
)";

// Recursive function to parse submenu
void parseSubmenu(JsonArray submenuArray, std::vector<MenuItem> &submenu) {
  for (JsonObject menuObj : submenuArray) {
    MenuItem item;
    item.path = menuObj["path"].as<std::string>();
    item.label = menuObj["label"].as<std::string>();

    // Parse Steps
    if (menuObj.containsKey("submenu")) {
      parseSubmenu(menuObj["submenu"].as<JsonArray>(), item.submenu);
    }

    // Parse steps
    if (menuObj.containsKey("steps")) {
      JsonArray stepsArray = menuObj["steps"].as<JsonArray>();
      for (JsonObject stepObj : stepsArray) {
        Step step;
        step.path = stepObj["path"].as<std::string>();
        step.label = stepObj["label"].as<std::string>();
        step.value = stepObj["value"] | 0;
        step.minValue = stepObj["minValue"] | 0;
        step.maxValue = stepObj["maxValue"] | 0;
        item.steps.push_back(step);
      }
    }

    submenu.push_back(item);
  }
}

// Function to parse JSON into vector<MenuItem>
// JsonArray parseMenuConfig() {
//   JsonDocument doc;

//   DeserializationError error = deserializeJson(doc, MENU_CONFIG);

//   if (error) {
//     Serial.print("OLED Json Parsing Failed: ");
//     while (true);
//   }

//   const JsonArray menuArray = doc[0]["submenu"].as<JsonArray>();

//   return menuArray;
// }

#include <ArduinoJson.h>

JsonDocument parseMenuConfig() {
  static JsonDocument doc;

  DeserializationError error = deserializeJson(doc, MENU_CONFIG);

  if (error) {
    Serial.print("OLED Json Parsing Failed: ");
    Serial.println(error.f_str());
    return JsonDocument();
  }

  return doc;
}