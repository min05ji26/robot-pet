void setup() {
  Serial.begin(9600);
}

void loop() {
  int peak = 0;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    int lo = 1023, hi = 0;
    unsigned long t = millis();
    while (millis() - t < 50) {
      int v = analogRead(A0);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    if (hi - lo > peak) peak = hi - lo;
  }
  Serial.println(peak);
}