// 통신 규칙 (docs/protocol.md와 항상 같게 유지)
#pragma once
#include <Arduino.h>

const uint8_t STX = 0x02;
const uint8_t ETX = 0x03;
const uint8_t PACKET_LEN = 6;  // [STX][CMD][D1][D2][CHK][ETX]

// LabVIEW → 아두이노
enum : uint8_t {
  CMD_STATE     = 0x01,
  CMD_SERVO     = 0x02,
  CMD_MELODY    = 0x03,
  CMD_THRESHOLD = 0x04,
  CMD_FACE      = 0x05,
  CMD_MOTOR     = 0x06,
  CMD_HEARTBEAT = 0x07,
};

// 아두이노 → LabVIEW
enum : uint8_t {
  RSP_TRIGGER   = 0x81,
  RSP_DISTANCE  = 0x82,
  RSP_BATTERY   = 0x83,
  RSP_DONE      = 0x84,
  RSP_ERROR     = 0x85,
  RSP_SAFE_STOP = 0x86,
};

// 0x85 D1: 오류 종류
enum : uint8_t { ERR_CHECKSUM = 1, ERR_ETX = 2, ERR_UNKNOWN_CMD = 3 };

// 0x86 D1: 안전정지 원인
enum : uint8_t { STOP_COMM_LOST = 1, STOP_NEAR_OBSTACLE = 2, STOP_LOW_BATTERY = 3 };

// FSM 상태 번호
enum : uint8_t { ST_IDLE = 0, ST_TRIGGERED, ST_REACT, ST_EXPLORE, ST_AVOID, ST_RETURN };

inline uint8_t checksum(uint8_t cmd, uint8_t d1, uint8_t d2) {
  return (uint8_t)(cmd + d1 + d2);
}
