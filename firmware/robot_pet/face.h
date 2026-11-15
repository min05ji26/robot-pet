// 표정 그리기 (OLED SSD1306 128×64, I2C A4/A5)
// - U8g2 _1_ 페이지 버퍼: 화면 전체(1KB) 대신 한 줄(128바이트)씩 그려서 SRAM을 아낀다
// - 표정 번호 = 상태 번호 (docs/protocol.md 4번 표), 번호 안의 움직임은 아두이노가 알아서 한다
//     0 대기     : 눈 깜빡깜빡 → 10초 지나면 잠(감은 눈 + z)
//                  복귀(5)에서 대기로 오면 2초 웃고 시작
//     1 트리거감지: 눈 커짐
//     2 반응     : 웃음
//     3 탐색이동 / 4 회피 / 5 복귀: 집중한 눈 + 입 꼼지락
// - 눈·입 모양은 HW 시험 코드(firmware/tests/servo_oled_test)의 face() 기준
// - 사용: 상태가 바뀔 때 drawFace(번호), loop()에서 updateFace(millis())
// - Wokwi(firmware/wokwi/face_test)에서도 이 파일을 그대로 써서 모양을 다듬는다
#pragma once
#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const unsigned long BLINK_PERIOD_MS = 3000;  // 3초마다
const unsigned long BLINK_CLOSED_MS = 150;   //   0.15초 감았다 뜸
const unsigned long SLEEP_AFTER_MS = 10000;  // 대기 10초 지나면 잠
const unsigned long ARRIVE_SMILE_MS = 2000;  // 복귀 → 대기 때 웃는 시간
const unsigned long WIGGLE_MS = 250;         // 입 꼼지락 간격

// 화면 한 장(프레임) 종류
enum : uint8_t { F_OPEN, F_BLINK, F_SLEEP, F_BIG, F_SMILE, F_FOCUS_A, F_FOCUS_B, F_BLANK };

uint8_t faceNum = 0;              // 지금 표정 번호 (0~5)
unsigned long faceStartMs = 0;    // 이 표정이 시작된 시각
bool arriveSmile = false;         // 복귀 → 대기로 와서 웃는 중인지
int16_t shownFrame = -1;          // 화면에 그려져 있는 프레임 (바뀔 때만 다시 그림)

void faceBegin() {
  u8g2.begin();
  u8g2.setBusClock(400000);  // I2C 빠르게: 한 번 그리는 시간을 줄여 패킷 받기가 밀리지 않게
}

// ---- 모양 조각 (HW face() 좌표 기준: 눈 중심 40,28 / 88,28) ----
void smallMouth() {  // ∨ 입
  u8g2.drawLine(52, 50, 64, 56);
  u8g2.drawLine(64, 56, 76, 50);
}

void openEyes() {
  u8g2.drawDisc(40, 28, 10);
  u8g2.drawDisc(88, 28, 10);
}

void closedEyes() {
  u8g2.drawBox(30, 27, 20, 3);
  u8g2.drawBox(78, 27, 20, 3);
}

void drawFrame(uint8_t f) {
  u8g2.firstPage();
  do {
    switch (f) {
      case F_OPEN:   // 대기: 뜬 눈
        openEyes();
        smallMouth();
        break;
      case F_BLINK:  // 대기: 깜빡 (감은 눈)
        closedEyes();
        smallMouth();
        break;
      case F_SLEEP:  // 잠: 감은 눈 + z
        closedEyes();
        u8g2.drawLine(58, 52, 70, 52);
        u8g2.drawLine(108, 6, 118, 6);
        u8g2.drawLine(118, 6, 108, 16);
        u8g2.drawLine(108, 16, 118, 16);
        break;
      case F_BIG:    // 트리거감지: 커진 눈 + 반짝 + o 입
        u8g2.drawDisc(40, 28, 15);
        u8g2.drawDisc(88, 28, 15);
        u8g2.setDrawColor(0);
        u8g2.drawDisc(45, 22, 4);
        u8g2.drawDisc(93, 22, 4);
        u8g2.setDrawColor(1);
        u8g2.drawCircle(64, 55, 4);
        break;
      case F_SMILE:  // 반응·도착: 웃는 눈 ^ ^ + 활짝 웃는 입
        u8g2.drawLine(30, 32, 40, 22); u8g2.drawLine(40, 22, 50, 32);
        u8g2.drawLine(78, 32, 88, 22); u8g2.drawLine(88, 22, 98, 32);
        u8g2.drawCircle(64, 42, 12, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
        break;
      case F_FOCUS_A:  // 이동 중: 또렷하게 뜬 눈 + 반짝 + 안쪽으로 내린 눈썹(집중) + 입 ①
      case F_FOCUS_B:  //                                                       + 입 ②
        // 눈: 위를 자르지 않고 동그랗게 다 뜬다 (전에는 위를 잘라서 졸린 눈처럼 보였음)
        u8g2.drawDisc(40, 30, 11);
        u8g2.drawDisc(88, 30, 11);
        u8g2.setDrawColor(0);
        u8g2.drawDisc(44, 26, 3);  // 반짝 (F_BIG보다 작게)
        u8g2.drawDisc(92, 26, 3);
        u8g2.setDrawColor(1);
        // 눈썹: 바깥은 높고 안쪽은 낮게 → 앞을 노려보는 집중 표정 (2px 두께)
        u8g2.drawLine(28, 11, 50, 15); u8g2.drawLine(28, 12, 50, 16);
        u8g2.drawLine(78, 15, 100, 11); u8g2.drawLine(78, 16, 100, 12);
        if (f == F_FOCUS_A) {
          u8g2.drawLine(56, 52, 72, 52);
        } else {
          u8g2.drawLine(56, 52, 60, 50); u8g2.drawLine(60, 50, 64, 52);
          u8g2.drawLine(64, 52, 68, 50); u8g2.drawLine(68, 50, 72, 52);
        }
        break;
      default:  // F_BLANK: 없는 번호면 빈 화면
        break;
    }
  } while (u8g2.nextPage());
}

// 지금 시각에 보여야 할 프레임
uint8_t frameFor(unsigned long now) {
  unsigned long t = now - faceStartMs;
  switch (faceNum) {
    case 0:
      if (arriveSmile && t < ARRIVE_SMILE_MS) return F_SMILE;
      if (t >= SLEEP_AFTER_MS) return F_SLEEP;
      return (t % BLINK_PERIOD_MS) >= BLINK_PERIOD_MS - BLINK_CLOSED_MS ? F_BLINK : F_OPEN;
    case 1:
      return F_BIG;
    case 2:
      return F_SMILE;
    case 3:
    case 4:
    case 5:
      return (t / WIGGLE_MS) % 2 ? F_FOCUS_B : F_FOCUS_A;
    default:
      return F_BLANK;
  }
}

// 프레임이 바뀌었을 때만 다시 그린다 (loop에서 계속 부름)
void updateFace(unsigned long now) {
  uint8_t f = frameFor(now);
  if (f != shownFrame) {
    drawFrame(f);
    shownFrame = f;
  }
}

// 표정 번호(0~5)를 바꾼다. 복귀(5)에서 대기(0)로 오면 잠깐 웃는다
void drawFace(uint8_t num) {
  arriveSmile = (faceNum == 5 && num == 0);
  faceNum = num;
  faceStartMs = millis();
  updateFace(faceStartMs);
}
