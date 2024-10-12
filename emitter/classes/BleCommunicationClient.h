#pragma once
#include <Arduino.h>
#include <BLEMidi.h>

#include <string>

class BleCommunicationClient {
 private:
 public:
  BleCommunicationClient();

  static void writeBleMidiMessage(const std::string messageType,
                                  const int controllerNumber,
                                  const int value,
                                  const int channel = 0) {
    if (messageType == "controlChange") {
      BLEMidiServer.controlChange(channel, controllerNumber, value);
    }

    // if (this->midiMessage == "pitchBend") {
    //   if (this->currentValue != this->previousValue) {
    //     BLEMidiServer.pitchBend(channel, this->lsb, this->msb);
    //   }
    // }

    // if (this->midiMessage == "gate") {
    //   if (this->toggleStatus != this->previousToggleStatus) {
    //     if (this->toggleStatus) {
    //       BLEMidiServer.noteOn(channel, char(60), char(127));
    //     } else {
    //       BLEMidiServer.noteOff(channel, char(60), char(127));
    //     }
    //   }
    // }
  }
};