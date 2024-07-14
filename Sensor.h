#ifndef Sensor_h
#define Sensor_h
#include <Arduino.h>
#include "MPU6050.h"
#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include <vector>
#include <map>
#include <string>
#include "Utils.h"
#include <algorithm>
#include <ArduinoJson.h>

struct SensorConfig {
  std::string sensorType;
  int controllerNumber;
  int pin;
  int intPin;
  int floorThreshold;
  int ceilThreshold;
  std::string messageType;
  int amountOfReads;
};

class Sensor {
private:
  uint8_t controllerNumber;
  char _channel;
  uint8_t _statusCode;
  uint8_t measuresCounter;
  bool isActive;
  bool toggleStatus;
  bool previousToggleStatus;
  bool isAlreadyPressed;
  unsigned long dataBuffer;
  uint16_t _debounceThreshold;
  unsigned long _currentDebounceValue;
  unsigned long _previousDebounceValue;
  unsigned long currentDebounceTimer;
  unsigned long previousDebounceTimer;
  bool previousSwitchState;
  bool currentSwitchState;
  bool isDebounced;
  int16_t _threshold;
  int16_t _floor;
  int16_t _ceil;
  uint16_t getDebounceThreshold(std::string &type);
  uint8_t msb = 0;
  uint8_t lsb = 0;
  uint8_t counter = 0;
public:
  /**
     * @brief Constructs a Sensor object.
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
  // const int amountOfReads = pin["amountOfReads"];
  // const int floorThreshold = pin["floorThreshold"];
  // const int ceilThreshold = pin["ceilThreshold"];
  Sensor(const SensorConfig &config);
  std::string sensorType;
  std::string _midiMessage;
  uint8_t filteredExponentialValue;
  uint8_t pin;
  uint8_t intPin;
  uint8_t previousValue;
  int16_t previousRawValue;
  uint8_t currentValue;
  int16_t filteredValue;
  bool isSwitchActive();
  bool isAboveThreshold();
  bool isSwitchDebounced();
  int getMappedMidiValue(int16_t actualValue, int floor = 0, int ceil = 0);
  int16_t getRawValue(Adafruit_VL53L0X *lox);
  int16_t runNonBlockingAverageFilter();
  int16_t runBlockingAverageFilter(int measureSize, Adafruit_VL53L0X *lox, int gap = 500);
  int16_t runExponentialFilter(Adafruit_VL53L0X *lox);
  bool isSibling(const std::vector<std::string> &SIBLINGS);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentDebounceValue(unsigned long timeValue);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setPreviousRawValue(int16_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);
  void setThreshold(uint8_t value);
  void setThresholdBasedOnActiveSiblings(const uint8_t &amountOfActiveSiblings);
  void setMidiMessage(std::string value);
  void sendSerialMidiMessage(HardwareSerial *Serial5);
  void setMidiChannel(uint8_t channel);
  void debounce(Adafruit_VL53L0X *lox);
  void run(Adafruit_VL53L0X *lox);
  std::string getSensorType();

  static void setUpSensorPins(std::vector<Sensor *> SENSORS) {
    for (Sensor *SENSOR : SENSORS) {
      if (!!SENSOR->pin) {
        pinMode(SENSOR->pin, INPUT);
      }
      if (!!SENSOR->intPin) {
        pinMode(SENSOR->intPin, INPUT);
      }
    }
  }

  static std::vector<std::string> getSupportedSensors() {
    return { "infrared", "potentiometer", "force", "sonar", "ax", "ay" };
  }

  static std::map<std::string, HardwareSerial *> getSupportedSerialPorts(const std::string microcontroller) {
    std::map<std::string, HardwareSerial *> ports;

    if (microcontroller == "teensy") {
      ports.insert({ "Serial1", &Serial1 });
      ports.insert({ "Serial2", &Serial2 });
      ports.insert({ "Serial3", &Serial3 });
      ports.insert({ "Serial4", &Serial4 });
      ports.insert({ "Serial5", &Serial5 });
    } else {
      return std::map<std::string, HardwareSerial *>();
    }
  }

  static std::vector<Sensor *>
  initializeSensors(const char *configJson) {

    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, configJson);

    if (error) {
      doc.clear();
    }

    std::vector<Sensor *> sensors;

    JsonArray pinsArray = doc["sensors"].as<JsonArray>();

    const int hardwareSerialBaudRate = doc.containsKey("hardwareSerialBaudRate") ? doc["hardwareSerialBaudRate"] : 230400;

    if (hardwareSerialBaudRate) {
      Serial.begin(hardwareSerialBaudRate);
      std::string successMessage = "Successfully initialized hardware serial port with the speed: " + std::to_string(hardwareSerialBaudRate);

      Serial.println(successMessage.c_str());
    } else {
      Serial.println("Failed to initialize the hardware serial port");
      while (true)
        ;
      delay(1);
    }

    const std::vector<std::string> validSensors = Sensor::getSupportedSensors();

    // const std::map<std::string, HardwareSerial *> = Sensor::getSupportedSerialPorts()

    for (JsonObject pinObj : pinsArray) {
      const std::string sensorType = pinObj["sensorType"];
      const int controllerNumber = pinObj["controllerNumber"];
      const int pin = pinObj["pin"];
      const int intPin = pinObj["intPin"];
      const int amountOfReads = pinObj["filter"]["amountOfReads"];
      const int floorThreshold = pinObj["floorThreshold"];
      const int ceilThreshold = pinObj["ceilThreshold"];
      const std::string messageType = pinObj["messageType"];

      const boolean isSensorNotSupported = std::find(validSensors.begin(), validSensors.end(), sensorType) == validSensors.end();

      if (isSensorNotSupported) {
        const std::string message = "sensor " + sensorType + " is not supported...";
        Serial.println(message.c_str());
        continue;
      }

      SensorConfig config = {
        sensorType,
        controllerNumber,
        pin,
        intPin,
        floorThreshold,
        ceilThreshold,
        messageType,
        amountOfReads,
      };

      Sensor *sensor = new Sensor(config);

      sensors.push_back(sensor);
    }

    return sensors;
  }

  static Sensor *getSensorBySensorType(std::vector<Sensor *> SENSORS, std::string sensorType) {
    for (Sensor *SENSOR : SENSORS) {
      if (SENSOR->sensorType == sensorType) {
        return SENSOR;
      }
    }
    const std::string errorMessage = "Sensor " + sensorType + " not found";
    Serial.println(errorMessage.c_str());
  }

  static bool isPitchButtonActive(bool &currentButtonState, bool &lastButtonState, bool &toggleStatus, const uint8_t &PITCH_BEND_BUTTON) {
    currentButtonState = !!digitalRead(PITCH_BEND_BUTTON);
    if (currentButtonState && !lastButtonState) {
      toggleStatus = !toggleStatus ? true : false;
    }
    lastButtonState = currentButtonState;
    return toggleStatus;
  }

  static void setInfraredSensorStates(Sensor *infraredSensor, bool &pitchBendLedState, int16_t thresholdValue, std::string midiMessage, bool newLedState, const uint8_t &PITCH_BEND_LED) {
    pitchBendLedState = newLedState;
    digitalWrite(PITCH_BEND_LED, pitchBendLedState);
    infraredSensor->setThreshold(thresholdValue);
    infraredSensor->setMidiMessage(midiMessage);
  }

  static void runPitchBendLogic(Sensor *infraredSensor, const bool &isBendActive, bool &pitchBendLedState, const uint8_t &PITCH_BEND_LED) {
    if (isBendActive && !pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 2, "pitchBend", true, PITCH_BEND_LED);
    }
    if (!isBendActive && pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 1, "controlChange", false, PITCH_BEND_LED);
    }
  }

  static void testAccelgiroConnection(MPU6050 &accelgyro) {
    accelgyro.initialize();
    if (accelgyro.testConnection()) {
      Serial.println("Succesfully connected to IMU!");
    } else {
      Serial.println("There was a problem with the IMU initialization");
    }
    delay(100);
  }

  static void testInfraredSensorConnection(Adafruit_VL53L0X &lox, uint8_t i2c_addr, const uint8_t &ERROR_LED, TwoWire *i2c = &Wire) {
    if (!lox.begin(i2c_addr, false, &Wire)) {
      Serial.println("Failed to boot VL53L0X");
    } else {
      Serial.println("Succesfully connected to VL53L0X!");
    }
    delay(100);
  }

  static bool is_active(Sensor *SENSOR, std::vector<std::string> listOfCandidates) {
    if (std::find(listOfCandidates.begin(), listOfCandidates.end(), SENSOR->sensorType) != listOfCandidates.end()) {
      return SENSOR->isSwitchActive();
    }
    return false;
  }

  static uint8_t getActiveSiblings(std::vector<Sensor *> SENSORS, std::vector<std::string> candidates) {
    const uint8_t activeSiblings = std::count_if(SENSORS.begin(), SENSORS.end(), [&](Sensor *s) {
      return is_active(s, candidates);
    });
    return activeSiblings;
  }

  static bool is_debounced(Sensor *SENSOR, std::vector<std::string> listOfCandidates) {
    if (std::find(listOfCandidates.begin(), listOfCandidates.end(), SENSOR->sensorType) != listOfCandidates.end()) {
      return SENSOR->isSwitchDebounced();
    }
    return false;
  }

  static uint8_t areAllSiblingsDebounced(std::vector<Sensor *> SENSORS, std::vector<std::string> candidates) {
    const uint8_t amountOfDebouncedSensors = std::count_if(SENSORS.begin(), SENSORS.end(), [&](Sensor *s) {
      return is_debounced(s, candidates);
    });
    return candidates.size() == amountOfDebouncedSensors;
  }

  // static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue, HardwareSerial *Serial2) {
  //   static const byte rightGuillemet[] = { 0xC2, 0xBB };  //UTF-8 character for separating MIDI messages: 11000010, 10111011
  //   Serial2->write(char(statusCode));
  //   Serial2->write(char(controllerNumber));
  //   Serial2->write(char(sensorValue));
  //   Serial2->write(rightGuillemet, sizeof(rightGuillemet));
  // }

  /**
  * Check if this approach is noticeable faster than the one above
  **/
  static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue, HardwareSerial *Serial5) {
    Utils::printMidiMessage(statusCode, controllerNumber, sensorValue);
    uint16_t rightGuillemet = 0xBB00 | 0xC2;  // combine the two bytes into a single uint16_t value
    Serial5->write(&statusCode, 1);
    Serial5->write(&controllerNumber, 1);
    Serial5->write(&sensorValue, 1);
    Serial5->write(reinterpret_cast<uint8_t *>(&rightGuillemet), 2);  // reinterpret the uint16_t value as a byte array and send 2 bytes
  }
};


#endif