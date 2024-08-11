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
  { 1, IMU_BASE_FILTER_THRESHOLD }, { 2, 30 }, { 3, 25 }, { 4, 20 }, { 5, 15 }, { 6, 5 },
};

uint16_t MidiSensor::getDebounceThreshold(std::string &type) {
  static std::map<std::string, int> debounceThresholdValues = {
    { "force", 30 },
  };
  return debounceThresholdValues[type];
}

MidiSensor::MidiSensor(const SensorConfig &config) {
  accelgyro = config.accelgyro;
  infraredSensor = config.infraredSensor;
  sensorType = config.sensorType;
  controllerNumber = config.controllerNumber;
  pin = config.pin;
  midiBus = config.midiBus;
  filterType = config.filterType;
  filterWeight = config.filterWeight ? config.filterWeight : 1;
  writeContinousValues = config.writeContinousValues;
  midiCommunicationType = config.midiCommunicationType;
  intPin = config.intPin;
  floorThreshold = config.floorThreshold;
  ceilThreshold = config.ceilThreshold;
  midiMessage = config.messageType;
  channel = 0;
  statusCode = config.statusCode;
  previousValue = 0;
  currentRawValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
  filteredExponentialValue = 0;
  averageValue = 0;
  currentDebounceValue = 0;
  previousDebounceValue = 0;
  isActive = false;
  toggleStatus = false;
  previousToggleStatus = toggleStatus;
  isAlreadyPressed = false;
  debounceThreshold = MidiSensor::getDebounceThreshold(sensorType);
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
  return this->measuresCounter % this->filterWeight == 0;
};

void MidiSensor::setCurrentValue(uint8_t value) {
  this->currentValue = value;
}

void MidiSensor::setThreshold(uint8_t value) {
  this->filterWeight = value;
}

void MidiSensor::setThresholdBasedOnActiveSiblings(const uint8_t &amountOfActiveSiblings) {
  if (this->sensorType == "ax" || this->sensorType == "ay") {
    this->filterWeight = imuFilterResolution[amountOfActiveSiblings];
  }
}

void MidiSensor::setMidiMessage(std::string value) {
  this->midiMessage = value;
}

void MidiSensor::setPreviousValue(uint8_t value) {
  this->previousValue = value;
}

void MidiSensor::setCurrentDebounceValue(unsigned long timeValue) {
  this->currentDebounceValue = timeValue;
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

int16_t MidiSensor::getCurrentValue() {
  std::map<std::string, std::function<int16_t()>> measureMethods;
  measureMethods["potentiometer"] = [this]() { return analogRead(pin); };
  measureMethods["force"] = [this]() { return analogRead(pin); };
  measureMethods["sonar"] = [this]() {
    const uint32_t pulse = pulseIn(pin, HIGH);
    const int16_t inches = pulse / 147;
    return inches;
  };
  measureMethods["accelgyro_ax"] = [this]() -> int16_t {
    if (!!this->accelgyro) {
      return this->accelgyro->getAccelerationX();
    }

    return 0;
  };
  measureMethods["accelgyro_ay"] = [this]() -> int16_t {
    if (!!this->accelgyro) {
      return this->accelgyro->getAccelerationY();
    }

    return 0;
  };
  measureMethods["accelgyro_az"] = [this]() -> int16_t {
    if (!!this->accelgyro) {
      return this->accelgyro->getAccelerationZ();
    }

    return 0;
  };
  measureMethods["accelgyro_gx"] = [this]() -> int16_t {
    if (!!this->accelgyro) {
      return this->accelgyro->getRotationX();
    }

    return 0;
  };
  measureMethods["accelgyro_gy"] = [this]() -> int16_t {
    if (!!this->accelgyro) {
      return this->accelgyro->getRotationY();
    }

    return 0;
  };
  measureMethods["accelgyro_gz"] = [this]() -> int16_t {
    if (!!this->accelgyro) {
      return this->accelgyro->getRotationZ();
    }

    return 0;
  };
  measureMethods["infrared"] = [this]() {
    if (!!this->infraredSensor) {
      VL53L0X_RangingMeasurementData_t measure;

      this->infraredSensor->rangingTest(&measure, false);

      const bool isMesureAboveThreshold = measure.RangeStatus != 4 && measure.RangeMilliMeter >= this->floorThreshold / 2;

      return isMesureAboveThreshold ? measure.RangeMilliMeter : this->currentRawValue;
    }

    return 0;
  };

  const auto it = measureMethods.find(this->sensorType);
  const auto hasMatchingFunction = it != measureMethods.end();

  if (hasMatchingFunction) {
    return it->second();
  }

  const std::string message = "|| Can't get the raw value for the sensor type \"" + this->sensorType + "\"";
  Serial.println(message.c_str());

  return 0;
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

int MidiSensor::getMappedMidiValue(int16_t actualValue) {
  if (this->midiMessage == "pitchBend") {
    const int pitchBendValue = constrain(map(actualValue, this->floorThreshold, this->ceilThreshold, 8191, 16383), 8191, 16383);
    int shiftedValue = pitchBendValue << 1;
    this->msb = highByte(shiftedValue);
    this->lsb = lowByte(shiftedValue) >> 1;
    return pitchBendValue;
  }

  return constrain(map(actualValue, this->floorThreshold, this->ceilThreshold, 0, 127), 0, 127);
}

void MidiSensor::debounce() {
  if (sensorType == "force") {
    this->previousToggleStatus = this->toggleStatus;
    if (this->currentDebounceValue - this->previousDebounceValue >= debounceThreshold) {
      this->currentRawValue = this->getCurrentValue();
      const uint8_t sensorMappedValue = this->getMappedMidiValue(this->currentRawValue);
      this->toggleStatus = !!sensorMappedValue ? true : false;
      this->previousDebounceValue = this->currentDebounceValue;
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

  for (uint8_t &step : steps) {
    startValue += delta;
    MidiSensor::writeSerialMidiMessage(this->statusCode, this->controllerNumber, startValue);
    step = startValue;
  }
}

void MidiSensor::sendSerialMidiMessage() {
  if (midiCommunicationType != "serial") {
    const std::string message = "Communication type \"" + midiCommunicationType + "\" is not yet supported.";
    Serial.println(message.c_str());
    return;
  }

  if (this->midiMessage == "controlChange" && this->currentValue != this->previousValue) {
    if (this->writeContinousValues) {
      this->writeContinousMessages();
    } else {
      this->writeSerialMidiMessage(this->statusCode, this->controllerNumber, this->currentValue);
    }
  }
  if (this->midiMessage == "gate" && this->toggleStatus != this->previousToggleStatus) {
    if (this->toggleStatus) {
      this->writeSerialMidiMessage(144, 60, 127);
    } else {
      this->writeSerialMidiMessage(128, 60, 127);
    }
  }
}

int16_t MidiSensor::runNonBlockingAverageFilter() {
  return this->dataBuffer / this->filterWeight;
}

int16_t MidiSensor::runExponentialFilter(float alpha) {
  const auto value = this->currentRawValue * alpha + (1.0f - alpha) * this->previousValue;
  return static_cast<int16_t>(value);
}

int16_t MidiSensor::runLowPassFilter() {
  this->averageValue += (this->currentRawValue - this->averageValue) / this->filterWeight;
  return static_cast<int16_t>(std::round(this->averageValue));
}

void MidiSensor::runCommonFilterLogic(const int16_t averageValue) {
  const uint8_t sensorMappedValue = this->getMappedMidiValue(averageValue);
  this->setPreviousValue(this->currentValue);
  this->setCurrentValue(sensorMappedValue);
  this->debounce();
}

void MidiSensor::runFilterLogic() {
  std::map<std::string, std::function<void()>> filterFunctions;
  filterFunctions["exponential"] = [this]() {
    const int16_t averageValue = this->runExponentialFilter(0.8f);
    this->runCommonFilterLogic(averageValue);
  };

  filterFunctions["lowPass"] = [this]() {
    const float averageValue = this->runLowPassFilter();
    this->runCommonFilterLogic(averageValue);
  };

  filterFunctions["averageNonBlocking"] = [this]() {
    this->setDataBuffer(this->currentRawValue);
    this->setMeasuresCounter(1);
    if (this->isAboveThreshold()) {
      const int16_t averageValue = this->runNonBlockingAverageFilter();
      this->runCommonFilterLogic(averageValue);
      this->setMeasuresCounter(0);
      this->setDataBuffer(0);
    }
  };

  const auto filterFunctionToRun = filterFunctions.find(this->filterType);
  const auto hasMatchingFunction = filterFunctionToRun != filterFunctions.end();

  if (!hasMatchingFunction) {
    const std::string message =
        "|| Skipping sensor \"" + this->sensorType + "\" Since filter type \"" + this->filterType + "\" is not yet implemented.";
    Serial.println(message.c_str());
  }

  filterFunctionToRun->second();
}

void MidiSensor::run() {
  this->currentRawValue = this->getCurrentValue();
  this->setCurrentDebounceValue(millis());
  this->runFilterLogic();
  this->sendSerialMidiMessage();
}

/**
 * TODO: Test for ESP32 device.
 **/
// void MidiSensor::sendBleMidiMessage(BLEMidiServerClass *serverInstance) {
//   if (this->midiMessage == "controlChange") {
//     if (this->currentValue != this->previousValue) {
//       serverInstance->controlChange(channel, controllerNumber, char(this->currentValue));
//     }
//   }
// if (this->midiMessage == "gate") {
//   if (this->toggleStatus != this->previousToggleStatus) {
//     if (this->toggleStatus) {
//       serverInstance->noteOn(channel, char(60), char(127));
//     } else {
//       serverInstance->noteOff(channel, char(60), char(127));
//     }
//   }
// }
//   if (this->midiMessage == "pitchBend") {
//     if (this->currentValue != this->previousValue) {
//       serverInstance->pitchBend(channel, this->lsb, this->msb);
//     }
//   }
// }