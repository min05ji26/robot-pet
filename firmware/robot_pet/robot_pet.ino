// 펫로봇 펌웨어 (2단계부터 채워 나감)
// - 통신 값은 docs/protocol.md(통신 규칙표 v1.0) 기준
// - 통신 경로는 패킷 전용: 디버그 출력(Serial.print) 금지 (LabVIEW 수신이 깨짐)
//
// 진행 (노션 2단계 SW)
//   [x] receivePacket() + 체크섬 검사 + 오류 시 0x85 전송
//   [x] handleCommand() 분기 뼈대 (0x01~0x07), 처리 후 0x84
//   [x] 주기 작업: 초음파 200ms → 0x82, 배터리 1초 → 0x83
//   [x] 하트비트 1초 끊기면 모터 정지 + 0x86
//   [x] 근접 10cm 이하 시 아두이노가 즉시 모터 정지 + 0x86 (LabVIEW 판단 없이)
//
// 진행 (노션 3단계 SW)
//   [x] 표정 그리기 drawFace(번호): face.h (U8g2 _1_ 페이지 버퍼)
//       + 대기 깜빡임·10초 뒤 잠, 이동 중 입 꼼지락, 복귀 끝나면 웃음 (updateFace)
//   [x] 멜로디 playMelody(번호): melody.h (PROGMEM, 1 깸 / 2 기쁨 / 3 잘 자), 끝나면 0x84
//   [x] 자체 소음 블랭킹: 모터·서보·부저 동작 중 + 끝난 뒤 300ms 마이크 무시 (+ 박수 감지 → 0x81)
//   [x] String 안 쓰기, SRAM 75% 이하 (String·Serial.print 없음, SRAM 921B 44%)

#include "protocol.h"
#include "face.h"  // 표정 drawFace() — 라이브러리 매니저에서 U8g2 설치 필요
#include "melody.h"  // 멜로디 playMelody() — 부저 D12
#include <Adafruit_PWMServoDriver.h>  // 서보 PCA9685 — 라이브러리 매니저에서 Adafruit PWM Servo Driver 설치 필요

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

uint8_t currentState = ST_IDLE;   // LabVIEW가 0x01로 알려 주는 현재 상태 (0~5)
uint16_t clapThreshold = 200;     // 박수 임계값 (마이크 3.3V, 50ms 최댓값−최솟값 기준 측정값)
// 하트비트 감시: LabVIEW가 0x07을 200~300ms마다 보낸다. 1초 넘게 안 오면 통신 끊김
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;
unsigned long lastHeartbeatMs = 0;  // 마지막 0x07 받은 시각
bool heartbeatSeen = false;   // 첫 하트비트 전에는 끊김 판단 안 함
bool commLostSent = false;    // 한 번 끊길 때 0x86은 한 번만 보냄
bool safeStopped = false;     // 안전 정지 중: 0x06(모터) 무시, 0x01로 대기(0)·복귀(5)를 받으면 풀림

// 근접 정지: 10cm 이하면 LabVIEW 판단 없이 바로 멈춤
const uint8_t NEAR_STOP_CM = 10;
bool nearSent = false;        // 가까이 있는 동안 0x86은 한 번만 (10cm보다 멀어지면 다시 보낼 수 있음)

// 센서 핀 (HW 배선 기준)
const uint8_t TRIG_PIN = 10, ECHO_PIN = 11;  // 초음파 HC-SR04
const uint8_t BATT_PIN = A1;                 // 배터리 분압(10kΩ×2)

// 주기 작업 간격
const unsigned long DIST_PERIOD_MS = 200;
const unsigned long BATT_PERIOD_MS = 1000;
unsigned long lastDistMs = 0, lastBattMs = 0;

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

// ---- 부품 함수 (HW 시험 코드에서 가져옴) ----

// 서보: firmware/tests/servo_oled_test (PCA9685 0x40, 50Hz, SERVOMIN 150 / SERVOMAX 600)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
const uint16_t SERVOMIN = 150, SERVOMAX = 600;

void servoBegin() {
  pca.begin();
  pca.setPWMFreq(50);
  for (uint8_t ch = 0; ch < 3; ch++) pca.setPWM(ch, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));  // 시작은 가운데(90도)
}



void setAngle(uint8_t ch, uint8_t angle) {
  if (ch > 2) return;  // 서보는 채널 0~2만 사용
  pca.setPWM(ch, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
}

// 모터: firmware/tests/motor_functions (L298N, 초음파 쪽이 앞 기준으로 방향 확인 완료)
//   IN1~4 → D2/D3/D4/D7, ENA/ENB → D5/D6 (Timer0 PWM이라 AltSoftSerial·tone과 안 겹침)
const uint8_t ENA = 5, IN1 = 2, IN2 = 3;
const uint8_t ENB = 6, IN3 = 4, IN4 = 7;

void motorBegin() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void motor(uint8_t a1, uint8_t a2, uint8_t b1, uint8_t b2, uint8_t spd) {
  digitalWrite(IN1, a1); digitalWrite(IN2, a2);
  digitalWrite(IN3, b1); digitalWrite(IN4, b2);
  analogWrite(ENA, spd); analogWrite(ENB, spd);
}

void forward(uint8_t spd)   { motor(1, 0, 1, 0, spd); }
void backward(uint8_t spd)  { motor(0, 1, 0, 1, spd); }
void turnLeft(uint8_t spd)  { motor(1, 0, 0, 1, spd); }
void turnRight(uint8_t spd) { motor(0, 1, 1, 0, spd); }
void stopMotor()            { motor(0, 0, 0, 0, 0); }

// ---- 박수 감지 + 자체 소음 블랭킹 ----
// 박수: 50ms 동안 마이크 최댓값−최솟값이 임계값(200)을 넘으면 0x81 (HW mic_test 측정 방식)
// 블랭킹: 로봇 자기 소리(모터·서보·부저)를 박수로 착각하지 않게,
//         그것들이 동작하는 동안 + 끝난 뒤 300ms는 마이크를 무시한다
const uint8_t MIC_PIN = A0;
const unsigned long CLAP_WINDOW_MS = 50;      // 이 시간 동안의 최댓값−최솟값을 본다
const unsigned long CLAP_COOLDOWN_MS = 500;   // 박수 한 번이 두 번 잡히지 않게
const unsigned long BLANK_AFTER_MS = 300;     // 동작이 끝난 뒤에도 300ms는 무시
const unsigned long SERVO_MOVE_MS = 500;      // 서보 명령 뒤 움직이는 시간으로 본다

bool motorRunning = false;          // 모터가 도는 중인지
unsigned long quietAfterMs = 0;     // 이 시각 전까지는 마이크 무시
int micLo = 1023, micHi = 0;        // 지금 50ms 동안의 최솟값·최댓값
unsigned long micWindowStart = 0;
bool micWindowNoisy = false;        // 이번 50ms 중에 자기 소리가 있었는지
bool clapSeen = false;
unsigned long lastClapMs = 0;

// 마이크를 무시할 시각을 t까지 늘린다
void blankUntil(unsigned long t) {
  if ((long)(t - quietAfterMs) > 0) quietAfterMs = t;
}

// 지금 로봇이 자기 소리를 내고 있는지 (모터·부저 동작 중, 또는 끝난 지 300ms 안)
bool selfNoise(unsigned long now) {
  return motorRunning || melodyPlaying() || (long)(quietAfterMs - now) > 0;
}

// 모터 상태를 기록한다. 멈추면 그때부터 300ms 블랭킹
void setMotorRunning(bool on) {
  if (motorRunning && !on) blankUntil(millis() + BLANK_AFTER_MS);
  motorRunning = on;
}

void checkClap(unsigned long now) {
  int v = analogRead(MIC_PIN);
  if (v < micLo) micLo = v;
  if (v > micHi) micHi = v;
  if (selfNoise(now)) micWindowNoisy = true;
  if (now - micWindowStart < CLAP_WINDOW_MS) return;

  int peak = micHi - micLo;
  bool noisy = micWindowNoisy;
  micLo = 1023; micHi = 0; micWindowStart = now; micWindowNoisy = false;  // 다음 50ms 시작

  if (noisy || peak <= (int)clapThreshold) return;            // 자기 소리였거나 작은 소리
  if (clapSeen && now - lastClapMs < CLAP_COOLDOWN_MS) return; // 방금 박수와 같은 소리
  clapSeen = true;
  lastClapMs = now;
  uint16_t loud = peak / 4;
  sendPacket(RSP_TRIGGER, loud > 255 ? 255 : loud, 0);  // 0x81: D1 = 소리 크기÷4
}

// 받은 명령을 CMD별로 나눠 처리하고, 끝나면 0x84(D1 = 완료한 CMD)를 보낸다
//   0x07 하트비트는 0x84를 보내지 않는다 / 모르는 CMD는 0x85 원인 3
void handleCommand(uint8_t cmd, uint8_t d1, uint8_t d2) {
  switch (cmd) {
    case CMD_STATE:  // 0x01 상태 변경: D1 = 상태 번호(0~5), 표정도 같은 번호로
      if (d1 > ST_RETURN) return;  // 없는 상태 번호는 무시
      currentState = d1;
      if (d1 == ST_IDLE || d1 == ST_RETURN) safeStopped = false;  // 대기·복귀를 받으면 안전 정지 풀림
      drawFace(d1);
      break;
    case CMD_SERVO:  // 0x02 서보: D1 = 채널(0~2), D2 = 각도(0~180)
      if (d2 > 180) d2 = 180;
      setAngle(d1, d2);
      blankUntil(millis() + SERVO_MOVE_MS + BLANK_AFTER_MS);  // 서보 소리 블랭킹
      break;
    case CMD_MELODY:  // 0x03 멜로디: D1 = 번호 (0 멈춤 / 1 깸 / 2 기쁨 / 3 잘 자)
      if (playMelody(d1)) return;  // 재생이 끝날 때 loop()에서 0x84를 보냄
      break;                       // 멈춤·없는 번호는 바로 0x84
    case CMD_THRESHOLD:  // 0x04 박수 임계값: D1 = 값÷4
      clapThreshold = d1 * 4;
      break;
    case CMD_FACE:  // 0x05 표정: D1 = 표정 번호(0~5)
      drawFace(d1);
      break;
    case CMD_MOTOR:  // 0x06 모터: D1 = 0 정지 / 1 전진 / 2 후진 / 3 좌회전 / 4 우회전, D2 = 속도
      if (safeStopped) return;  // 안전 정지 중에는 모터 명령 무시 (0x84 안 보냄)
      switch (d1) {
        case 1: forward(d2); break;
        case 2: backward(d2); break;
        case 3: turnLeft(d2); break;
        case 4: turnRight(d2); break;
        default: stopMotor(); break;
      }
      setMotorRunning(d1 >= 1 && d1 <= 4);  // 모터 소리 블랭킹
      break;
    case CMD_HEARTBEAT:  // 0x07 하트비트: 받은 시각만 기록, 0x84 없음
      lastHeartbeatMs = millis();
      heartbeatSeen = true;
      commLostSent = false;  // 다시 연결됨 (안전 정지는 0x01 대기·복귀로만 풀림)
      return;
    default:
      sendPacket(RSP_ERROR, ERR_UNKNOWN_CMD, 0);
      return;
  }
  sendPacket(RSP_DONE, cmd, 0);
}

// ---- 센서 읽기 (HW 측정 코드 firmware/tests/ultrasonic_test, battery_test에서 가져옴) ----

// 거리(cm). 30ms 안에 메아리가 없으면(약 5m 넘음) 255 = 최대
uint8_t readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long t = pulseIn(ECHO_PIN, HIGH, 30000);
  if (t == 0) return 255;
  unsigned long cm = (t * 17 + 500) / 1000;  // t × 0.034 ÷ 2, 반올림
  return cm > 255 ? 255 : cm;
}

// 배터리 전압 × 10 (예: 8.4V → 84). 분압이라 측정값 × 2
uint8_t readBatteryX10() {
  unsigned long raw = analogRead(BATT_PIN);
  return (raw * 100 + 511) / 1023;  // raw × 5 ÷ 1023 × 2 × 10, 반올림
}

// 10cm 이하면 모터를 멈추고 0x86(원인 1 근접)을 보낸다
void checkNear(uint8_t cm) {
  if (cm <= NEAR_STOP_CM) {
    if (nearSent) return;
    stopMotor();
    setMotorRunning(false);
    safeStopped = true;
    nearSent = true;
    sendPacket(RSP_SAFE_STOP, STOP_NEAR_OBSTACLE, 0);
  } else {
    nearSent = false;
  }
}

// 정해진 시간마다 센서 값을 LabVIEW로 보낸다 (delay 없이 millis로 재서 패킷 받기를 막지 않음)
void periodicTasks(unsigned long now) {
  if (now - lastDistMs >= DIST_PERIOD_MS) {
    lastDistMs = now;
    uint8_t cm = readDistanceCm();
    sendPacket(RSP_DISTANCE, cm, 0);  // 0x82: D1 = cm
    checkNear(cm);
  }
  if (now - lastBattMs >= BATT_PERIOD_MS) {
    lastBattMs = now;
    sendPacket(RSP_BATTERY, readBatteryX10(), 0);   // 0x83: D1 = 전압 × 10
  }
}

// 하트비트가 1초 넘게 끊기면 모터를 멈추고 0x86(원인 2 통신 끊김)을 보낸다
void checkHeartbeat(unsigned long now) {
  if (!heartbeatSeen || commLostSent) return;
  if (now - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    stopMotor();
    setMotorRunning(false);
    safeStopped = true;
    commLostSent = true;
    sendPacket(RSP_SAFE_STOP, STOP_COMM_LOST, 0);
  }
}

void setup() {
  LINK.begin(BAUD);
  motorBegin();
  stopMotor();        // 켜자마자 바퀴가 돌지 않게
  faceBegin();
  servoBegin();       // faceBegin() 다음: 같은 I2C(A4/A5)를 OLED와 나눠 씀
  drawFace(ST_IDLE);  // 전원 켜면 대기(감은 눈)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  uint8_t cmd, d1, d2;
  if (receivePacket(cmd, d1, d2)) {
    handleCommand(cmd, d1, d2);
  }
  unsigned long now = millis();
  periodicTasks(now);
  checkHeartbeat(now);
  updateFace(now);  // 표정 움직임 (깜빡임·잠·입 꼼지락)
  if (updateMelody(now)) {  // 멜로디가 끝나면 0x84 + 부저 소리 블랭킹 300ms
    sendPacket(RSP_DONE, CMD_MELODY, 0);
    blankUntil(now + BLANK_AFTER_MS);
  }
  checkClap(now);
}
