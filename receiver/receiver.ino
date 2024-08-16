void setup() {
  Serial3.begin(230400);
  Serial1.begin(31250);
}

void loop() {
  int chars = Serial3.available();

  if (!!chars) {
    String midiMessage = Serial3.readStringUntil('»');
    MIDImessage(midiMessage.charAt(0), midiMessage.charAt(1), midiMessage.charAt(2));
  }
}

void MIDImessage(byte command, byte data1, byte data2) {
  Serial1.write(command);
  Serial1.write(data1);
  Serial1.write(data2);
}