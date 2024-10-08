void setup() {
  Serial3.begin(230400);
  Serial1.begin(31250);
  Serial.begin(9600);
  Serial.println("|| Starting receiver program...");
}

void loop() {
  const int chars = Serial3.available();

  if (!!chars) {
    String midiMessage = Serial3.readStringUntil('»');

    const unsigned char command = midiMessage.charAt(0);
    const unsigned char data1 = midiMessage.charAt(1);
    const unsigned char data2 = midiMessage.charAt(2);

    // printMessage(command, data1, data2);

    /**
     * TODO: Remove this check after fixing message sent in emitter for zero values
     */
    if (data1 <= 127) {
      sendMIDIMessage(command, data1, data2);
    }
  }
}

void sendMIDIMessage(char command, char data1, char data2) {
  // Send MIDI data via DIN cable
  sendSerialMIDIMessage(command, data1, data2);
  // Send MIDI data via USB Serial
  sendUSBMIDIMessage(command, data1, data2);
}

void sendSerialMIDIMessage(char command, char data1, char data2) {
  Serial1.write(command);
  Serial1.write(data1);
  Serial1.write(data2);
}

void sendUSBMIDIMessage(char command, char data1, char data2) {
  usbMIDI.sendControlChange(command, data1, data2);
}

void printMessage(char command, char data1, char data2) {
  Serial.println("--------------------");
  Serial.println((int)command);  // Controller number (e.g., Control Change)
  Serial.println((int)data1);    // Controller value or note velocity
  Serial.println((int)data2);    // Channel
}
