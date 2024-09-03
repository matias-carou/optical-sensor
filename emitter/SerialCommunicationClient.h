#ifndef SerialCommunicationClient_h
#define SerialCommunicationClient_h
#include <Arduino.h>

#include <string>

#include "Config.h"

class SerialCommunicationClient {
 private:
 public:
  SerialCommunicationClient();

  static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue, auto serialBus) {
    uint16_t rightGuillemet = 0xBB00 | 0xC2;  // combine the two bytes into a
    serialBus->write(&statusCode, 1);
    serialBus->write(&controllerNumber, 1);
    serialBus->write(&sensorValue, 1);
    serialBus->write(reinterpret_cast<uint8_t *>(&rightGuillemet), 2);
  }
};

#endif