#include "MidiSensor.h"

#include <math.h>

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "Adafruit_VL53L0X.h"
#include "MPU6050.h"

static const int IMU_BASE_FILTER_THRESHOLD = 35;

static std::map<uint8_t, uint8_t> imuFilterResolution = {
    {1, IMU_BASE_FILTER_THRESHOLD}, {2, 30}, {3, 25}, {4, 20}, {5, 15}, {6, 5},
};

uint16_t MidiSensor::getDebounceThreshold(std::string &type) {
  static std::map<std::string, int> debounceThresholdValues = {
      {"force", 30},
  };
  return debounceThresholdValues[type];
}

MidiSensor::MidiSensor(const SensorConfig &config) {
  sensorType = config.sensorType;
  controllerNumber = config.controllerNumber;
  pin = config.pin;
  midiBus = config.midiBus;
  filterType = config.filterType;
  communicationType = config.communicationType;
  intPin = config.intPin;
  _floor = config.floorThreshold;
  _ceil = config.ceilThreshold;
  _threshold = config.amountOfReads ? config.amountOfReads : 1;
  _midiMessage = config.messageType;
  _channel = 0;
  _statusCode = 176;
  previousValue = 0;
  previousRawValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
  filteredExponentialValue = 0;
  _currentDebounceValue = 0;
  _previousDebounceValue = 0;
  isActive = false;
  toggleStatus = false;
  previousToggleStatus = toggleStatus;
  isAlreadyPressed = false;
  _debounceThreshold = MidiSensor::getDebounceThreshold(sensorType);
  currentDebounceTimer = 0;
  previousDebounceTimer = 0;
  currentSwitchState = false;
  previousSwitchState = false;
  isDebounced = false;
  counter = 0;
  msb = 0;
  lsb = 0;
}

bool MidiSensor::isAboveThreshold() {
  return this->measuresCounter % this->_threshold == 0;
};

void MidiSensor::setCurrentValue(uint8_t value) {
  this->currentValue = value;
}

void MidiSensor::setThreshold(uint8_t value) {
  this->_threshold = value;
}

void MidiSensor::setThresholdBasedOnActiveSiblings(const uint8_t &amountOfActiveSiblings) {
  if (this->sensorType == "ax" || this->sensorType == "ay") {
    this->_threshold = imuFilterResolution[amountOfActiveSiblings];
  }
}

void MidiSensor::setMidiMessage(std::string value) {
  this->_midiMessage = value;
}

void MidiSensor::setPreviousValue(uint8_t value) {
  this->previousValue = value;
}

void MidiSensor::setPreviousRawValue(int16_t value) {
  this->previousRawValue = value;
}

void MidiSensor::setMidiChannel(uint8_t channel) {
  this->_channel = channel;
}

void MidiSensor::setCurrentDebounceValue(unsigned long timeValue) {
  this->_currentDebounceValue = timeValue;
}

void MidiSensor::setMeasuresCounter(uint8_t value) {
  this->measuresCounter = !value ? value : this->measuresCounter + value;
}

void MidiSensor::setDataBuffer(int16_t value) {
  this->dataBuffer = !value ? value : this->dataBuffer + value;
}

std::string MidiSensor::getSensorType() {
  return this->sensorType;
}

bool MidiSensor::isSwitchActive() {
  const bool isSwitchActive = !!this->intPin ? !!digitalRead(this->intPin) : true;
  this->currentSwitchState = isSwitchActive;
  if (this->currentSwitchState != this->previousSwitchState) {
    this->isDebounced = false;
  }
  return isSwitchActive;
}

bool MidiSensor::isSwitchDebounced() {
  this->currentDebounceTimer = millis();
  if (sensorType == "ax" || sensorType == "ay") {
    if (!this->isDebounced) {
      if (this->currentDebounceTimer - this->previousDebounceTimer >= 500) {
        this->previousSwitchState = this->currentSwitchState;
        this->isDebounced = true;
        this->previousDebounceTimer = this->currentDebounceTimer;
        return true;
      }
      return false;
    }
    this->previousDebounceTimer = this->currentDebounceTimer;
    return true;
  }
  return true;
}

int16_t MidiSensor::getRawValue(Adafruit_VL53L0X *lox) {
  if (sensorType == "potentiometer" || sensorType == "force") {
    return analogRead(pin);
  }

  // if (sensorType == "ax") {
  //   const int16_t rawValue = accelgyro->getAccelerationX();
  //   return constrain(rawValue, 0, _ceil);
  // }

  // if (sensorType == "ay") {
  //   const int16_t rawValue = accelgyro->getAccelerationY();
  //   return constrain(rawValue, 0, _ceil);
  // }

  // if (sensorType == "az") {
  //   const int16_t rawValue = accelgyro->getAccelerationZ();
  //   return constrain(rawValue, 0, _ceil);
  // }

  if (sensorType == "sonar") {
    const uint32_t pulse = pulseIn(pin, HIGH);
    const int16_t inches = pulse / 147;
    return inches;
  }

  // if (sensorType == "gx") {
  //   const int16_t rawValue = accelgyro->getRotationX();
  //   return constrain(rawValue, 0, _ceil);
  // }

  // if (sensorType == "gy") {
  //   const int16_t rawValue = accelgyro->getRotationY();
  //   return constrain(rawValue, 0, _ceil);
  // }

  // if (sensorType == "gz") {
  //   const int16_t rawValue = accelgyro->getRotationZ();
  //   return constrain(rawValue, 0, _ceil);
  // }

  if (sensorType == "infrared") {
    VL53L0X_RangingMeasurementData_t measure;

    lox->rangingTest(&measure, false);

    return measure.RangeStatus != 4 && measure.RangeMilliMeter >= _floor / 2 ? measure.RangeMilliMeter :
                                                                               this->previousRawValue;
  }

  return 0;
}

int16_t MidiSensor::runNonBlockingAverageFilter() {
  return this->dataBuffer / this->_threshold;
}

int16_t MidiSensor::runBlockingAverageFilter(int measureSize, Adafruit_VL53L0X *lox, int gap) {
  int buffer = 0;
  for (int i = 0; i < measureSize; i++) {
    int16_t value = this->getRawValue(lox);
    if (value < 0) {
      value = 0;
    }
    buffer += value;
    delayMicroseconds(gap);
  }
  const int16_t result = buffer / measureSize;
  return result;
}

int16_t MidiSensor::runExponentialFilter(Adafruit_VL53L0X *lox, int16_t rawValue, float alpha) {
  const auto value = rawValue * alpha + (1.0f - alpha) * this->previousValue;
  return static_cast<int16_t>(value);
}

std::vector<uint8_t> MidiSensor::getValuesBetweenRanges(uint8_t gap) {
  uint8_t samples = 1;
  if (currentValue > previousValue) {
    samples = currentValue - previousValue;
  }
  if (currentValue < previousValue) {
    samples = previousValue - currentValue;
  }
  std::vector<uint8_t> steps(samples / gap);
  uint8_t startValue = previousValue;
  std::generate(steps.begin(), steps.end(), [&startValue, this, &gap]() {
    if (this->currentValue > this->previousValue) {
      return startValue += gap;
    }
    if (this->currentValue < this->previousValue) {
      return startValue -= gap;
    }
    return startValue;
  });
  return steps;
}

int MidiSensor::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  if (floor && ceil) {
    return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
  }
  if (this->_midiMessage == "pitchBend") {
    const int pitchBendValue = constrain(map(actualValue, _floor, _ceil, 8191, 16383), 8191, 16383);
    int shiftedValue = pitchBendValue << 1;
    this->msb = highByte(shiftedValue);
    this->lsb = lowByte(shiftedValue) >> 1;
    return pitchBendValue;
  }
  return constrain(map(actualValue, _floor, _ceil, 0, 127), 0, 127);
}

void MidiSensor::debounce(Adafruit_VL53L0X *lox) {
  if (sensorType == "force") {
    this->previousToggleStatus = this->toggleStatus;
    if (this->_currentDebounceValue - this->_previousDebounceValue >= _debounceThreshold) {
      const int16_t rawValue = this->getRawValue(lox);
      const uint8_t sensorMappedValue = this->getMappedMidiValue(rawValue);
      this->toggleStatus = !!sensorMappedValue ? true : false;
      this->_previousDebounceValue = this->_currentDebounceValue;
    }
  }
}

bool MidiSensor::isSibling(const std::vector<std::string> &SIBLINGS) {
  const std::vector<std::string>::const_iterator it = std::find(SIBLINGS.begin(), SIBLINGS.end(), this->sensorType);
  return it != SIBLINGS.end();
}

void MidiSensor::writeContinousMessages() {
  uint8_t samples = std::abs(currentValue - previousValue);
  std::vector<uint8_t> steps(samples);
  uint8_t startValue = previousValue;

  int delta = (currentValue > previousValue) ? 1 : -1;

  for (auto &step : steps) {
    startValue += delta;
    MidiSensor::writeSerialMidiMessage(this->_statusCode, this->controllerNumber, startValue);
    step = startValue;
  }
}

void MidiSensor::sendSerialMidiMessage() {
  if (this->_midiMessage == "controlChange" && this->currentValue != this->previousValue) {
    if (communicationType == "continous") {
      this->writeContinousMessages();
    } else {
      MidiSensor::writeSerialMidiMessage(this->_statusCode, this->controllerNumber, this->currentValue);
    }
  }
  if (this->_midiMessage == "gate" && this->toggleStatus != this->previousToggleStatus) {
    if (this->toggleStatus) {
      MidiSensor::writeSerialMidiMessage(144, 60, 127);
    } else {
      MidiSensor::writeSerialMidiMessage(128, 60, 127);
    }
  }
}

void MidiSensor::run(Adafruit_VL53L0X *lox) {
  int16_t rawValue = this->getRawValue(lox);
  this->setPreviousRawValue(rawValue);
  this->setDataBuffer(rawValue);
  this->setMeasuresCounter(1);
  const unsigned long currentDebounceValue = millis();
  this->setCurrentDebounceValue(currentDebounceValue);
  if (filterType == "exponential") {
    const int16_t averageValue = this->runExponentialFilter(lox, rawValue, 0.8f);
    const uint8_t sensorMappedValue = this->getMappedMidiValue(averageValue);
    this->setPreviousValue(this->currentValue);
    this->setCurrentValue(sensorMappedValue);
    this->sendSerialMidiMessage();
  } else if (filterType == "averageNonBlocking") {
    if (this->isAboveThreshold()) {
      const int16_t averageValue = this->runNonBlockingAverageFilter();
      const uint8_t sensorMappedValue = this->getMappedMidiValue(averageValue);
      this->setPreviousValue(this->currentValue);
      this->setCurrentValue(sensorMappedValue);
      this->debounce(lox);
      this->sendSerialMidiMessage();
      this->setMeasuresCounter(0);
      this->setDataBuffer(0);
    }
  } else {
    const std::string message = "Filter type \"" + filterType + "\" is not supported";
    Serial.println(message.c_str());
  }
}

/**
 * TODO: Test for ESP32 device.
 **/
// void MidiSensor::sendBleMidiMessage(BLEMidiServerClass *serverInstance) {
//   if (this->_midiMessage == "controlChange") {
//     if (this->currentValue != this->previousValue) {
//       serverInstance->controlChange(_channel, controllerNumber, char(this->currentValue));
//     }
//   }
// if (this->_midiMessage == "gate") {
//   if (this->toggleStatus != this->previousToggleStatus) {
//     if (this->toggleStatus) {
//       serverInstance->noteOn(_channel, char(60), char(127));
//     } else {
//       serverInstance->noteOff(_channel, char(60), char(127));
//     }
//   }
// }
//   if (this->_midiMessage == "pitchBend") {
//     if (this->currentValue != this->previousValue) {
//       serverInstance->pitchBend(_channel, this->lsb, this->msb);
//     }
//   }
// }