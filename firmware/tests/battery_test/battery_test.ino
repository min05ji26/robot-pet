void setup() { Serial.begin(9600); }
void loop() {
  float v = analogRead(A1) * 5.0 / 1023.0 * 2;
  Serial.println(v);
  delay(1000);
}