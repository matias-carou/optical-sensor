#include "ActionService.h"

#include "MidiSensor.h"
#include "types.h"

std::map<std::string, std::function<void(MidiSensor *, SelectedOption &)>> ActionService::sensorActions{
  { "root.sensors.imu.floor",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int modeValue = std::stoi(option.value);
      sensor->setFloor(modeValue);
    } },
  { "root.sensors.imu.ceil",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int modeValue = std::stoi(option.value);
      sensor->setCeil(modeValue);
    } },
  { "root.sensors.imu.filter.weight",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int weightValue = std::stoi(option.value);
      sensor->setFilterWeight(weightValue);
    } },
  { "root.sensors.imu.mode",
    [](MidiSensor *sensor, SelectedOption &option) {
      // TODO: Make this better
      if (option.value == "gyroscope") {
        if (sensor->getSensorType() == "accelgyro_gx") {
          sensor->setSensorType("accelgyro_ax");
        }

        if (sensor->getSensorType() == "accelgyro_gy") {
          sensor->setSensorType("accelgyro_ay");
        }
      } else if (option.value == "accelerometer") {
        if (sensor->getSensorType() == "accelgyro_ax") {
          sensor->setSensorType("accelgyro_gx");
        }

        if (sensor->getSensorType() == "accelgyro_ay") {
          sensor->setSensorType("accelgyro_gy");
        }
      }
    } },
  { "root.sensors.tof.floor",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int modeValue = std::stoi(option.value);
      sensor->setFloor(modeValue);
    } },
  { "root.sensors.tof.ceil",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int modeValue = std::stoi(option.value);
      sensor->setCeil(modeValue);
    } },
  { "root.sensors.tof.filter.weight",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int weightValue = std::stoi(option.value);
      sensor->setFilterWeight(weightValue);
    } },
  { "root.sensors.potentiometer.floor",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int modeValue = std::stoi(option.value);
      sensor->setFloor(modeValue);
    } },
  { "root.sensors.potentiometer.ceil",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int modeValue = std::stoi(option.value);
      sensor->setCeil(modeValue);
    } },
  { "root.sensors.potentiometer.filter.weight",
    [](MidiSensor *sensor, SelectedOption &option) {
      const int weightValue = std::stoi(option.value);
      sensor->setFilterWeight(weightValue);
    } },
  { "root.config.heap", [](MidiSensor *sensor, SelectedOption &option) { Utils::printHeapInfo(500); } },
};

ActionService::ActionService() {
}

void ActionService::runSensors(std::vector<MidiSensor *> &sensors) {
  for (MidiSensor *SENSOR : sensors) {
    if (!SENSOR->isSwitchActive()) {
      continue;
    }

    SENSOR->run();
  }
}

void ActionService::dispatchAction(std::vector<MidiSensor *> &sensors, SelectedOption &selectedOption) {
  const std::string selectedId = selectedOption.id;
  const TDependency dependencies = selectedOption.dependencies;

  const std::vector<MidiSensor *> filteredSensors = MidiSensor::getSensorsByDependency(sensors, dependencies);

  display.getInstance().showText(selectedId.c_str());
  delay(500);

  if (selectedId == "root.config.heap") {
    display.getInstance().showText("Got heap");
    delay(500);
    const auto matchedFunction = sensorActions.find(selectedId);
    return matchedFunction->second(filteredSensors[0], selectedOption);
  }

  if (filteredSensors.empty()) {
    display.getInstance().showText("No Sensors Found");
    delay(500);
    return;
  }

  for (MidiSensor *sensor : filteredSensors) {
    const std::string sensorType = sensor->getSensorType();
    const auto matchedFunction = sensorActions.find(selectedId);

    if (matchedFunction == sensorActions.end()) {
      display.getInstance().showText("Action Not Supported");
      delay(500);
      return;
    }

    matchedFunction->second(sensor, selectedOption);
  }
}