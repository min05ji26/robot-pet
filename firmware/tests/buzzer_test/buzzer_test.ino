void setup() {
  int melody[] = {523, 659, 784, 1047};
  for (int i = 0; i < 4; i++) {
    tone(12, melody[i], 150);
    delay(200);
  }
}
void loop() {}