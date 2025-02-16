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
};

ActionService::ActionService() {
}

void ActionService::dispatchAction(std::vector<MidiSensor *> &sensors, SelectedOption &selectedOption) {
  const std::string selectedId = selectedOption.id;
  const TDependency dependencies = selectedOption.dependencies;

  const std::vector<MidiSensor *> filteredSensors = MidiSensor::getSensorsByDependency(sensors, dependencies);

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