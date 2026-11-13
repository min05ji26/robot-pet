const int ENA = 5, IN1 = 2, IN2 = 3;
const int ENB = 6, IN3 = 4, IN4 = 7;

void motor(int a1, int a2, int b1, int b2, int spd) {
  digitalWrite(IN1, a2); digitalWrite(IN2, a1);   // a1, a2 자리 바꿈
  digitalWrite(IN3, b2); digitalWrite(IN4, b1);   // b1, b2 자리 바꿈
  analogWrite(ENA, spd); analogWrite(ENB, spd);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void loop() {
  motor(1,0,1,0,180); delay(2000);  // 전진
  motor(0,0,0,0,0);   delay(1000);  // 정지
  motor(0,1,0,1,180); delay(2000);  // 후진
  motor(0,0,0,0,0);   delay(1000);  // 정지
}