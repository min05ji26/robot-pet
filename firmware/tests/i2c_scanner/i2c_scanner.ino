#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(500);
  Serial.println("I2C 스캔 시작");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("발견: 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("끝");
}

void loop() {}