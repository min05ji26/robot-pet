// 펫로봇 펌웨어 (2단계부터 채워 나감)
// - 통신 값은 docs/protocol.md(통신 규칙표 v1.0) 기준
// - 통신 경로는 패킷 전용: 디버그 출력(Serial.print) 금지 (LabVIEW 수신이 깨짐)
//
// 진행 (노션 2단계 SW)
//   [x] receivePacket() + 체크섬 검사 + 오류 시 0x85 전송
//   [ ] handleCommand() 분기 뼈대 (0x01~0x07), 처리 후 0x84
//   [ ] 주기 작업: 초음파 200ms → 0x82, 배터리 1초 → 0x83
//   [ ] 하트비트 1초 끊기면 모터 정지 + 0x86

#include "protocol.h"

// 통신 경로 선택
//   0 = USB 케이블 (Serial)          ← LabVIEW와 USB로 시험할 때
//   1 = 블루투스 HC-05 (D8 RX, D9 TX) ← 실제 로봇
#define USE_BT 0

#if USE_BT
#include <AltSoftSerial.h>
AltSoftSerial bt;  // RX=D8, TX=D9 고정
#define LINK bt
#else
#define LINK Serial
#endif

const unsigned long BAUD = 9600;  // HC-05 기본값, LabVIEW Configure Port와 같게

uint8_t rxBuf[PACKET_LEN];
uint8_t rxLen = 0;

void sendPacket(uint8_t cmd, uint8_t d1, uint8_t d2) {
  uint8_t pkt[PACKET_LEN] = {STX, cmd, d1, d2, checksum(cmd, d1, d2), ETX};
  LINK.write(pkt, PACKET_LEN);
}

// 버퍼 안에서 다음 STX를 찾아 앞으로 당긴다. 없으면 비운다
// (데이터 안에 0x02가 섞여 있을 수 있어서, 틀린 패킷이면 STX 하나만 버리고 다시 찾는다)
void resync() {
  for (uint8_t i = 1; i < PACKET_LEN; i++) {
    if (rxBuf[i] == STX) {
      memmove(rxBuf, rxBuf + i, PACKET_LEN - i);
      rxLen = PACKET_LEN - i;
      return;
    }
  }
  rxLen = 0;
}

// 정상 패킷 하나를 받으면 true. 기다리지 않는다(블로킹 없음)
//   STX 찾기 → 6바이트 모으기 → ETX 확인 → 체크섬 확인
//   틀리면 0x85 전송: D1 = 원인(1 체크섬 / 2 ETX 없음), D2 = 0
bool receivePacket(uint8_t &cmd, uint8_t &d1, uint8_t &d2) {
  while (LINK.available()) {
    uint8_t b = LINK.read();
    if (rxLen == 0 && b != STX) continue;  // STX 전의 쓰레기 값은 버림
    rxBuf[rxLen++] = b;
    if (rxLen < PACKET_LEN) continue;      // 6바이트가 다 올 때까지 기다림

    if (rxBuf[5] != ETX) {
      sendPacket(RSP_ERROR, ERR_ETX, 0);
      resync();
      continue;
    }
    if (rxBuf[4] != checksum(rxBuf[1], rxBuf[2], rxBuf[3])) {
      sendPacket(RSP_ERROR, ERR_CHECKSUM, 0);
      resync();
      continue;
    }
    cmd = rxBuf[1];
    d1 = rxBuf[2];
    d2 = rxBuf[3];
    rxLen = 0;
    return true;
  }
  return false;
}

void setup() {
  LINK.begin(BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  uint8_t cmd, d1, d2;
  if (receivePacket(cmd, d1, d2)) {
    // 정상 패킷이면 D13 LED 깜빡임 (다음 항목 handleCommand()로 바꿀 자리)
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
