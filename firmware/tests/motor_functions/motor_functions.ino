const int ENA = 5, IN1 = 2, IN2 = 3;
const int ENB = 6, IN3 = 4, IN4 = 7;

void motor(int a1, int a2, int b1, int b2, int spd) {
  digitalWrite(IN1, a1); digitalWrite(IN2, a2);
  digitalWrite(IN3, b1); digitalWrite(IN4, b2);
  analogWrite(ENA, spd); analogWrite(ENB, spd);
}

void forward(int spd)   { motor(1,0,1,0,spd); }
void backward(int spd)  { motor(0,1,0,1,spd); }
void turnLeft(int spd)  { motor(1,0,0,1,spd); }
void turnRight(int spd) { motor(0,1,1,0,spd); }
void stopMotor()        { motor(0,0,0,0,0); }

void setup() {
  Serial.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void loop() {
  Serial.println("전진");   forward(180);   delay(2000); stopMotor(); delay(1000);
  Serial.println("후진");   backward(180);  delay(2000); stopMotor(); delay(1000);
  Serial.println("좌회전"); turnLeft(180);  delay(2000); stopMotor(); delay(1000);
  Serial.println("우회전"); turnRight(180); delay(2000); stopMotor(); delay(3000);
}