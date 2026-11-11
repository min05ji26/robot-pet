#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_PWMServoDriver.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

void setAngle(int ch, int a) {
  pca.setPWM(ch, 0, map(a, 0, 180, 150, 600));
}

void face(bool open) {
  u8g2.firstPage();
  do {
    if (open) { u8g2.drawDisc(40, 28, 10); u8g2.drawDisc(88, 28, 10); }
    else      { u8g2.drawBox(30, 27, 20, 3); u8g2.drawBox(78, 27, 20, 3); }
    u8g2.drawLine(52, 50, 64, 56); u8g2.drawLine(64, 56, 76, 50);
  } while (u8g2.nextPage());
}

void setup() {
  u8g2.begin();
  pca.begin();
  pca.setPWMFreq(50);
}

void loop() {
  face(true);  setAngle(0, 30);  delay(1000);
  face(false); setAngle(0, 150); delay(1000);
}