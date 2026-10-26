// 가짜 아두이노: 센서·모터 없이 LabVIEW 통신만 시험하는 스케치
// - USB 시리얼(115200)로 동작하므로 블루투스 없이 LabVIEW VISA에서 COM 포트를 열면 된다
// - 시리얼은 패킷 전용: Serial.print 디버그 출력 금지 (LabVIEW 수신이 깨짐)
// - 받은 패킷이 정상이면 D13 LED가 깜빡인다
//
// 흉내 내는 것
//   0x81 박수: 10초마다
//   0x82 거리: 200ms마다, 100cm → 5cm → 100cm 왕복 (20cm 회피, 10cm 안전정지 시험)
//   0x83 배터리: 1초마다 8.40V에서 0.01V씩 감소, 6.50V 아래면 다시 8.40V
//   0x84 완료: 0x01~0x06 수신 시 바로 응답
//   0x85 오류: 체크섬·ETX 불일치, 모르는 명령
//   0x86 안전정지: 하트비트 1초 끊김 / 10cm 이하 / 6.6V 이하 (원인마다 진입 시 1회)

#include "protocol.h"

const unsigned long BAUD = 115200;
const unsigned long DIST_PERIOD_MS = 200;
const unsigned long BATT_PERIOD_MS = 1000;
const unsigned long CLAP_PERIOD_MS = 10000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

const uint16_t NEAR_STOP_CM = 10;
const uint16_t LOW_BATT_10MV = 660;  // 6.60V

uint8_t rxBuf[PACKET_LEN];
uint8_t rxLen = 0;

uint8_t currentState = ST_IDLE;
uint16_t clapThreshold = 600;

unsigned long lastDistMs = 0, lastBattMs = 0, lastClapMs = 0;
unsigned long lastHeartbeatMs = 0;
bool heartbeatSeen = false;  // 첫 하트비트 전에는 통신두절 판단 안 함

uint16_t fakeDistCm = 100;
int8_t distStep = -5;
uint16_t fakeBatt10mV = 840;

bool commLostSent = false, nearSent = false, lowBattSent = false;

void sendPacket(uint8_t cmd, uint8_t d1, uint8_t d2) {
  uint8_t pkt[PACKET_LEN] = {STX, cmd, d1, d2, checksum(cmd, d1, d2), ETX};
  Serial.write(pkt, PACKET_LEN);
}

void send16(uint8_t cmd, uint16_t value) {
  sendPacket(cmd, highByte(value), lowByte(value));
}

// 버퍼 안에서 다음 STX를 찾아 앞으로 당긴다. 없으면 비운다
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

// 정상 패킷 하나를 받으면 true. 블로킹하지 않는다
bool receivePacket(uint8_t &cmd, uint8_t &d1, uint8_t &d2) {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (rxLen == 0 && b != STX) continue;
    rxBuf[rxLen++] = b;
    if (rxLen < PACKET_LEN) continue;

    if (rxBuf[5] != ETX) {
      sendPacket(RSP_ERROR, ERR_ETX, rxBuf[1]);
      resync();
      continue;
    }
    if (rxBuf[4] != checksum(rxBuf[1], rxBuf[2], rxBuf[3])) {
      sendPacket(RSP_ERROR, ERR_CHECKSUM, rxBuf[1]);
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

void handleCommand(uint8_t cmd, uint8_t d1, uint8_t d2) {
  switch (cmd) {
    case CMD_HEARTBEAT:
      lastHeartbeatMs = millis();
      heartbeatSeen = true;
      commLostSent = false;
      return;  // 하트비트는 완료 알림 없음
    case CMD_STATE:
      currentState = d1;
      break;
    case CMD_THRESHOLD:
      clapThreshold = word(d1, d2);
      break;
    case CMD_SERVO:
    case CMD_MELODY:
    case CMD_FACE:
    case CMD_MOTOR:
      break;  // 가짜: 실제 동작 없이 바로 완료
    default:
      sendPacket(RSP_ERROR, ERR_UNKNOWN_CMD, cmd);
      return;
  }
  sendPacket(RSP_DONE, cmd, 0);
}

void updateFakeSensors(unsigned long now) {
  if (now - lastDistMs >= DIST_PERIOD_MS) {
    lastDistMs = now;
    fakeDistCm += distStep;
    if (fakeDistCm <= 5 || fakeDistCm >= 100) distStep = -distStep;
    send16(RSP_DISTANCE, fakeDistCm);

    if (fakeDistCm <= NEAR_STOP_CM && !nearSent) {
      sendPacket(RSP_SAFE_STOP, STOP_NEAR_OBSTACLE, fakeDistCm);
      nearSent = true;
    } else if (fakeDistCm > NEAR_STOP_CM) {
      nearSent = false;
    }
  }

  if (now - lastBattMs >= BATT_PERIOD_MS) {
    lastBattMs = now;
    fakeBatt10mV = (fakeBatt10mV <= 650) ? 840 : fakeBatt10mV - 1;
    send16(RSP_BATTERY, fakeBatt10mV);

    if (fakeBatt10mV <= LOW_BATT_10MV && !lowBattSent) {
      sendPacket(RSP_SAFE_STOP, STOP_LOW_BATTERY, fakeBatt10mV / 10);
      lowBattSent = true;
    } else if (fakeBatt10mV > LOW_BATT_10MV) {
      lowBattSent = false;
    }
  }

  if (now - lastClapMs >= CLAP_PERIOD_MS) {
    lastClapMs = now;
    send16(RSP_TRIGGER, clapThreshold + 100);
  }
}

void checkHeartbeat(unsigned long now) {
  if (heartbeatSeen && !commLostSent && now - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    sendPacket(RSP_SAFE_STOP, STOP_COMM_LOST, 0);
    commLostSent = true;
  }
}

void setup() {
  Serial.begin(BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  unsigned long now = millis();
  uint8_t cmd, d1, d2;

  if (receivePacket(cmd, d1, d2)) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    handleCommand(cmd, d1, d2);
  }
  updateFakeSensors(now);
  checkHeartbeat(now);
}
