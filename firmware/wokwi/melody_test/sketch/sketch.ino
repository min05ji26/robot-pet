// Wokwi 멜로디 시험: 1 깸 → 2 기쁨 → 3 잘 자를 차례로 들려 준다 (2초 쉬고 반복)
// 쓰는 법: wokwi.com에 이 sketch.ino와 ../diagram.json을 붙여 넣고
//          firmware/robot_pet/melody.h도 같은 이름의 파일로 추가한다 (브라우저 소리 켜기)
#include "melody.h"

uint8_t num = 1;
unsigned long waitUntil = 0;

void setup() {
  playMelody(num);
}

void loop() {
  unsigned long now = millis();
  if (updateMelody(now)) waitUntil = now + 2000;  // 끝나면 2초 쉼
  if (!melodyPlaying() && waitUntil && now >= waitUntil) {
    num = num % 3 + 1;
    playMelody(num);
    waitUntil = 0;
  }
}
