// 멜로디 (피에조 부저 D12)
// - 음표는 PROGMEM(플래시)에 저장해서 SRAM을 쓰지 않는다
// - 기다리지 않고 재생: playMelody(번호)로 시작, loop()에서 updateMelody(millis())
//   updateMelody()가 true를 돌려주면 막 끝난 것 → 0x84 완료 알림을 보낸다
// - 멜로디 번호 (0x03의 D1)
//     0 멈춤
//     1 깸     (트리거감지) : 도-미-솔-도 올라감 (HW buzzer_test와 같은 음)
//     2 기쁨   (반응)       : 짧고 통통 튀는 음
//     3 잘 자  (복귀)       : 도-솔-미-도 천천히 내려감
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>

const uint8_t BUZZER_PIN = 12;

// 음 높이(Hz)
// (A5 같은 이름은 아두이노 핀 이름과 겹쳐서 NOTE_를 붙인다)
const uint16_t NOTE_C5 = 523, NOTE_E5 = 659, NOTE_G5 = 784, NOTE_A5 = 880, NOTE_C6 = 1047, REST = 0;

// {음 높이, 길이(ms)} 쌍, 길이 0이 끝
const uint16_t MELODY_WAKE[] PROGMEM = {
  NOTE_C5, 150, NOTE_E5, 150, NOTE_G5, 150, NOTE_C6, 300,
  0, 0
};
const uint16_t MELODY_HAPPY[] PROGMEM = {
  NOTE_G5, 100, REST, 40, NOTE_G5, 100, NOTE_E5, 100, NOTE_G5, 100, NOTE_C6, 250, REST, 60, NOTE_A5, 120, NOTE_C6, 300,
  0, 0
};
const uint16_t MELODY_SLEEP[] PROGMEM = {
  NOTE_C6, 300, NOTE_G5, 300, NOTE_E5, 300, NOTE_C5, 600,
  0, 0
};

const uint16_t* const MELODIES[] = {nullptr, MELODY_WAKE, MELODY_HAPPY, MELODY_SLEEP};
const uint8_t MELODY_COUNT = 4;

const uint16_t* melodyNotes = nullptr;  // 재생 중인 멜로디 (없으면 nullptr)
uint8_t melodyIndex = 0;                // 몇 번째 음
unsigned long noteStartMs = 0;
uint16_t noteLenMs = 0;

bool melodyPlaying() {
  return melodyNotes != nullptr;
}

// index번째 음을 울린다. 끝이면 false
bool startNote(unsigned long now) {
  uint16_t freq = pgm_read_word(&melodyNotes[melodyIndex * 2]);
  uint16_t len = pgm_read_word(&melodyNotes[melodyIndex * 2 + 1]);
  if (len == 0) return false;
  if (freq == REST) noTone(BUZZER_PIN);
  else tone(BUZZER_PIN, freq);
  noteStartMs = now;
  noteLenMs = len;
  return true;
}

// 멜로디를 처음부터 시작한다. 0이나 없는 번호면 멈추고 false (바로 끝난 것으로 처리)
bool playMelody(uint8_t num) {
  noTone(BUZZER_PIN);
  melodyNotes = nullptr;
  if (num == 0 || num >= MELODY_COUNT) return false;
  melodyNotes = MELODIES[num];
  melodyIndex = 0;
  if (!startNote(millis())) {
    melodyNotes = nullptr;
    return false;
  }
  return true;
}

// 음 길이가 지나면 다음 음으로. 마지막 음이 끝난 그 순간에만 true
bool updateMelody(unsigned long now) {
  if (!melodyPlaying()) return false;
  if (now - noteStartMs < noteLenMs) return false;
  melodyIndex++;
  if (startNote(now)) return false;
  noTone(BUZZER_PIN);
  melodyNotes = nullptr;
  return true;
}
