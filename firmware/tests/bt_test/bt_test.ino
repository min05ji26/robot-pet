#include <AltSoftSerial.h>
AltSoftSerial bt;  // RX=D8, TX=D9 고정

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
}

void loop() {
  static unsigned long t = 0;
  if (millis() - t > 1000) {
    t = millis();
    bt.println("hello");
  }
  while (bt.available()) Serial.write(bt.read());
}