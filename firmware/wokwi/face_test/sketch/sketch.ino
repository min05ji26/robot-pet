// Wokwi 표정 시험: 상태가 바뀌는 순서대로 표정을 보여 준다
// 쓰는 법: wokwi.com에 이 sketch.ino, ../diagram.json, ../libraries.txt를 붙여 넣고
//          firmware/robot_pet/face.h도 같은 이름의 파일로 추가한다
//          (모양을 고쳤으면 face.h를 robot_pet 폴더에도 똑같이 반영)
#include "face.h"

// 순서: 대기(깜빡임 → 10초 뒤 잠) → 트리거감지 → 반응 → 탐색이동 → 회피 → 복귀 → 대기(웃고 → 깜빡임)
const uint8_t SEQ_FACE[] = {0, 1, 2, 3, 4, 5, 0};
const unsigned long SEQ_MS[] = {13000, 3000, 3000, 3000, 3000, 3000, 8000};
uint8_t step = 0;
unsigned long stepStart = 0;

void setup() {
  faceBegin();
  drawFace(SEQ_FACE[0]);
}

void loop() {
  unsigned long now = millis();
  if (now - stepStart >= SEQ_MS[step]) {
    step = (step + 1) % sizeof(SEQ_FACE);
    stepStart = now;
    drawFace(SEQ_FACE[step]);
  }
  updateFace(now);
}
