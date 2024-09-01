#ifndef Sensor_h
#define Sensor_h
#include <Arduino.h>
#include <ArduinoJson.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "Adafruit_VL53L0X.h"
#include "Config.h"
#include "MPU6050.h"
#include "Utils.h"
#include "Wire.h"

struct SensorConfig {
  std::string sensorType;
  int controllerNumber;
  int statusCode;
  int pin;
  int intPin;
  int floorThreshold;
  int ceilThreshold;
  std::string messageType;
  bool writeContinousValues;
  std::string filterType;
  int filterWeight;
  HardwareSerial *midiBus;
  Adafruit_VL53L0X *infraredSensor;
  MPU6050 *accelgyro;
  std::string midiCommunicationType;
};

class MidiSensor {
 private:
  std::map<std::string, std::function<int16_t()>> measureMethods;
  HardwareSerial *midiBus;
  Adafruit_VL53L0X *infraredSensor;
  MPU6050 *accelgyro;
  std::string filterType;
  int filterWeight;
  bool writeContinousValues;
  std::string midiCommunicationType;
  // uint8_t controllerNumber;
  char channel;
  // uint8_t statusCode;
  uint8_t measuresCounter;
  bool isActive;
  bool toggleStatus;
  bool previousToggleStatus;
  bool isAlreadyPressed;
  unsigned long dataBuffer;
  uint16_t debounceThreshold;
  unsigned long currentDebounceValue;
  unsigned long previousDebounceValue;
  unsigned long currentDebounceTimer;
  unsigned long previousDebounceTimer;
  bool previousSwitchState;
  bool currentSwitchState;
  bool isDebounced;
  int16_t floorThreshold;
  int16_t ceilThreshold;
  uint16_t getDebounceThreshold(std::string &type);
  std::map<std::string, std::function<int16_t()>> getMeasureMethods();
  uint8_t msb = 0;
  uint8_t lsb = 0;
  uint8_t counter = 0;

 public:
  /**
   * @brief Constructs a MidiSensor object.
   *
   * @param sensorType The type of sensor.
   * @param controllerNumber The controller number.
   * @param pin The analog/digital input pin, basically the sensor raw value.
   * @param intPin The on/off switch to initialize the sensor.
   * @param floorThreshold The reading floor for the sensor.
   * @param ceilThreshold The reading ceil for the sensor.
   * @param messageType The type of MIDI message.
   * @param amountOfReads
   */
  MidiSensor(const SensorConfig &config);
  uint8_t statusCode;
  uint8_t controllerNumber;
  uint8_t currentValue;
  std::string sensorType;
  std::string midiMessage;
  uint8_t filteredExponentialValue;
  uint8_t pin;
  uint8_t intPin;
  uint8_t previousValue;
  int16_t currentRawValue;
  int16_t filteredValue;
  float averageValue;
  bool isSwitchActive();
  bool isAboveThreshold();
  bool isSwitchDebounced();
  int getMappedMidiValue(int16_t actualValue);
  int16_t getCurrentValue();
  int16_t runNonBlockingAverageFilter();
  int16_t runExponentialFilter(const float alpha = 0.5f);
  int16_t runLowPassFilter();
  void runFilterLogic();
  void runCommonFilterLogic(const int16_t averageValue);
  bool isSibling(const std::vector<std::string> &SIBLINGS);
  std::vector<uint8_t> getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentDebounceValue(unsigned long timeValue);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);
  void setThreshold(uint8_t value);
  void setThresholdBasedOnActiveSiblings(const uint8_t &amountOfActiveSiblings);
  void setMidiMessage(std::string value);
  void sendSerialMidiMessage();
  void debounce();
  void run();
  void writeContinousMessages();
  std::string getSensorType();

  static void setUpSensorPins(std::vector<MidiSensor *> SENSORS) {
    for (MidiSensor *SENSOR : SENSORS) {
      if (!!SENSOR->pin) {
        pinMode(SENSOR->pin, INPUT);
      }
      if (!!SENSOR->intPin) {
        pinMode(SENSOR->intPin, INPUT);
      }
    }
  }

  static std::vector<std::string> getSupportedSensors() {
    return { "infrared",     "potentiometer", "force",        "sonar",        "accelgyro_ax",
             "accelgyro_ay", "accelgyro_az",  "accelgyro_gx", "accelgyro_gy", "accelgyro_gz" };
  }

  static std::map<std::string, HardwareSerial *> getSupportedSerialPorts() {
    std::map<std::string, HardwareSerial *> ports;

#if MICROCONTROLLER == MICROCONTROLLER_TEENSY
#ifdef Serial
    ports.insert({ "Serial", (HardwareSerial *)&Serial });
#endif
#ifdef Serial1
    ports.insert({ "Serial1", &Serial1 });
#endif
#ifdef Serial2
    ports.insert({ "Serial2", &Serial2 });
#endif
#ifdef Serial3
    ports.insert({ "Serial3", &Serial3 });
#endif
#ifdef Serial4
    ports.insert({ "Serial4", &Serial4 });
#endif
#ifdef Serial5
    ports.insert({ "Serial5", &Serial5 });
#endif
#elif MICROCONTROLLER == MICROCONTROLLER_ESP32
#ifdef Serial
    ports.insert({ "Serial", (HardwareSerial *)&Serial });
#endif

#else
    return std::map<std::string, HardwareSerial *>();
#endif

    return ports;
  }

  static std::vector<MidiSensor *> initializeSensors(const char *configJson) {
    Serial.println(F("|| Setting up sensors..."));

    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, configJson);

    if (error) {
      Serial.println("|| Failed to parse the JSON config...");
      while (true);
    }

    std::vector<MidiSensor *> sensors;

    const JsonArray pinsArray = doc["sensors"].as<JsonArray>();
    const JsonArray uartConfig = doc["uartConfig"].as<JsonArray>();

    if (!doc.containsKey("midiCommunicationType")) {
      Serial.println(F("No default communication type was provided in the config, setting \"serial\" as a fallback value..."));
    }

    const std::string midiCommunicationType =
        doc["midiCommunicationType"] ? doc["midiCommunicationType"].as<const char *>() : "serial";

    const std::map<std::string, HardwareSerial *> supportedPorts = MidiSensor::getSupportedSerialPorts();

    HardwareSerial *midiBus = nullptr;

    Serial.println("|| Setting up serial ports...");

    for (JsonObject uartObj : uartConfig) {
      const std::string port = uartObj["port"];
      const std::string communicationPurpose = uartObj["communicationPurpose"];
      const int baudRate = uartObj["baudRate"];
      const auto it = supportedPorts.find(port);
      if (it != supportedPorts.end()) {
        it->second->begin(baudRate);
        if (communicationPurpose == "midi") {
          midiBus = it->second;
        }
      }
    }

    if (!midiBus) {
      Serial.println("||No midi bus was configured for the instance...");
    }

    Wire.begin();

    Adafruit_VL53L0X *infraredSensor = nullptr;
    MPU6050 *accelgyro = nullptr;

    /**
     * Filter weight can vary depending on the filter type and input source
     */
    for (JsonObject pinObj : pinsArray) {
      const std::string sensorType = pinObj["sensorType"];
      const int controllerNumber = pinObj["controllerNumber"];
      const int statusCode = pinObj["statusCode"];
      const int pin = pinObj["inputPin"];
      const int intPin = pinObj["intPin"];
      const int floorThreshold = pinObj["floorThreshold"];
      const int ceilThreshold = pinObj["ceilThreshold"];
      const std::string messageType = pinObj["messageType"];
      const bool writeContinousValues = pinObj["writeContinousValues"] ? pinObj["writeContinousValues"] : false;
      const std::string filterType = pinObj["filter"]["type"];
      const int filterWeight = pinObj["filter"]["weight"];

      const std::vector<std::string> validSensors = MidiSensor::getSupportedSensors();
      const boolean isSensorNotSupported = std::find(validSensors.begin(), validSensors.end(), sensorType) == validSensors.end();

      if (isSensorNotSupported) {
        const std::string message = "Skipping sensor " + sensorType + " since is not supported...";
        Serial.println(message.c_str());
        continue;
      }

      if (!filterWeight) {
        const std::string message =
            "|| \"filterWeight\" key missing for sensor type \"" + sensorType + "\", setting fallback value...";
        Serial.println(message.c_str());
      }

      if (sensorType == "infrared" && !infraredSensor) {
        infraredSensor = new Adafruit_VL53L0X();

        if (!infraredSensor->begin(0x29, false, &Wire)) {
          Serial.println(F("|| Failed to boot VL53L0X"));
        } else {
          Serial.println(F("|| Successfully connected to VL53L0X!"));
        }
      }

      const bool isAccelgyroRelatedSensor = sensorType.find("accelgyro") != std::string::npos;

      if (isAccelgyroRelatedSensor && !accelgyro) {
        accelgyro = new MPU6050;
        accelgyro->initialize();

        if (accelgyro->testConnection()) {
          Serial.println(F("|| Successfully connected to IMU!"));
        } else {
          Serial.println(F("|| There was a problem with the IMU initialization"));
        }
      }

      SensorConfig config = { sensorType,     controllerNumber, statusCode,           pin,        intPin,       floorThreshold,
                              ceilThreshold,  messageType,      writeContinousValues, filterType, filterWeight, midiBus,
                              infraredSensor, accelgyro,        midiCommunicationType };

      MidiSensor *sensor = new MidiSensor(config);

      sensors.push_back(sensor);
    }

    MidiSensor::setUpSensorPins(sensors);

    return sensors;
  }

  static MidiSensor *getSensorBySensorType(std::vector<MidiSensor *> SENSORS, std::string sensorType) {
    for (MidiSensor *SENSOR : SENSORS) {
      if (SENSOR->sensorType == sensorType) {
        return SENSOR;
      }
    }
    const std::string errorMessage = "MidiSensor " + sensorType + " not found";
    Serial.println(errorMessage.c_str());
  }

  static bool isPitchButtonActive(bool &currentButtonState,
                                  bool &lastButtonState,
                                  bool &toggleStatus,
                                  const uint8_t &PITCH_BEND_BUTTON) {
    currentButtonState = !!digitalRead(PITCH_BEND_BUTTON);
    if (currentButtonState && !lastButtonState) {
      toggleStatus = !toggleStatus ? true : false;
    }
    lastButtonState = currentButtonState;
    return toggleStatus;
  }

  static void setInfraredSensorStates(MidiSensor *infraredSensor,
                                      bool &pitchBendLedState,
                                      int16_t thresholdValue,
                                      std::string midiMessage,
                                      bool newLedState,
                                      const uint8_t &PITCH_BEND_LED) {
    pitchBendLedState = newLedState;
    digitalWrite(PITCH_BEND_LED, pitchBendLedState);
    infraredSensor->setThreshold(thresholdValue);
    infraredSensor->setMidiMessage(midiMessage);
  }

  static void runPitchBendLogic(MidiSensor *infraredSensor,
                                const bool &isBendActive,
                                bool &pitchBendLedState,
                                const uint8_t &PITCH_BEND_LED) {
    if (isBendActive && !pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 2, "pitchBend", true, PITCH_BEND_LED);
    }
    if (!isBendActive && pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 1, "controlChange", false, PITCH_BEND_LED);
    }
  }

  static bool is_active(MidiSensor *SENSOR, std::vector<std::string> listOfCandidates) {
    if (std::find(listOfCandidates.begin(), listOfCandidates.end(), SENSOR->sensorType) != listOfCandidates.end()) {
      return SENSOR->isSwitchActive();
    }
    return false;
  }

  static uint8_t getActiveSiblings(std::vector<MidiSensor *> SENSORS, std::vector<std::string> candidates) {
    const uint8_t activeSiblings =
        std::count_if(SENSORS.begin(), SENSORS.end(), [&](MidiSensor *s) { return is_active(s, candidates); });
    return activeSiblings;
  }

  static bool is_debounced(MidiSensor *SENSOR, std::vector<std::string> listOfCandidates) {
    if (std::find(listOfCandidates.begin(), listOfCandidates.end(), SENSOR->sensorType) != listOfCandidates.end()) {
      return SENSOR->isSwitchDebounced();
    }
    return false;
  }

  static uint8_t areAllSiblingsDebounced(std::vector<MidiSensor *> SENSORS, std::vector<std::string> candidates) {
    const uint8_t amountOfDebouncedSensors =
        std::count_if(SENSORS.begin(), SENSORS.end(), [&](MidiSensor *s) { return is_debounced(s, candidates); });
    return candidates.size() == amountOfDebouncedSensors;
  }

  // static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue,
  // HardwareSerial *Serial2) {
  //   static const byte rightGuillemet[] = { 0xC2, 0xBB };  //UTF-8 character for separating MIDI messages: 11000010,
  //   10111011 Serial2->write(char(statusCode)); Serial2->write(char(controllerNumber));
  //   Serial2->write(char(sensorValue));
  //   Serial2->write(rightGuillemet, sizeof(rightGuillemet));
  // }

  /**
   * Check if this approach is noticeable faster than the one above
   **/
  void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue) {
    Utils::printMidiMessage(statusCode, controllerNumber, sensorValue);
    // uint16_t rightGuillemet = 0xBB00 | 0xC2;  // combine the two bytes into a
    // this->midiBus->write(&statusCode, 1);
    // this->midiBus->write(&controllerNumber, 1);
    // this->midiBus->write(&sensorValue, 1);
    // this->midiBus->write(reinterpret_cast<uint8_t *>(&rightGuillemet), 2);
  }
};

#endif