#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int ENA = 5, IN1 = 2, IN2 = 3;
const int ENB = 6, IN3 = 4, IN4 = 7;
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

void motor(int a1, int a2, int b1, int b2, int spd) {
  digitalWrite(IN1, a1); digitalWrite(IN2, a2);
  digitalWrite(IN3, b1); digitalWrite(IN4, b2);
  analogWrite(ENA, spd); analogWrite(ENB, spd);
}
void setAngle(int ch, int a) { pca.setPWM(ch, 0, map(a, 0, 180, 150, 600)); }

// 50ms 동안의 소리 크기
int level() {
  int lo = 1023, hi = 0;
  unsigned long t = millis();
  while (millis() - t < 50) {
    int v = analogRead(A0);
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi - lo;
}

// 3초 동안 측정, servoMove면 서보를 계속 움직임
void measure(const char* name, bool servoMove) {
  int peak = 0, angle = 30;
  unsigned long start = millis(), last = 0;
  while (millis() - start < 3000) {
    if (servoMove && millis() - last > 400) {
      angle = (angle == 30) ? 150 : 30;
      setAngle(0, angle);
      last = millis();
    }
    int v = level();
    if (v > peak) peak = v;
  }
  Serial.print(name); Serial.print(" 최대: "); Serial.println(peak);
}

void setup() {
  Serial.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pca.begin(); pca.setPWMFreq(50);
  setAngle(0, 90);
  delay(1000);
}

void loop() {
  motor(0,0,0,0,0);   measure("조용할 때", false);
  motor(1,0,1,0,180); measure("모터만   ", false);
  motor(0,0,0,0,0);   measure("서보만   ", true);
  motor(1,0,1,0,180); measure("모터+서보", true);
  motor(0,0,0,0,0);
  Serial.println("----");
  delay(3000);
}