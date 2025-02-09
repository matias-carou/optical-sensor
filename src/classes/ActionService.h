#ifndef ACTION_SERVICE_H
#define ACTION_SERVICE_H

#include "./types.h"
#include "DisplayManager.h"
#include "MidiSensor.h"

extern DisplayManager &display;

class ActionService {
 public:
  static ActionService &getInstance() {
    static ActionService instance;
    return instance;
  }

  void dispatchAction(std::vector<MidiSensor *> &sensors, SelectedOption &selectedOption);

 private:
  ActionService();  // Private constructor
  ~ActionService() = default;
  ActionService(const ActionService &) = delete;
  ActionService &operator=(const ActionService &) = delete;
  static std::map<std::string, std::function<void(MidiSensor *, SelectedOption &)>> sensorActions;
};

#endif
