# LabVIEW (1단계: 통신 뼈대)

`.vi` 파일은 이 폴더에 저장한다. 통신 값은 `docs/protocol.md` 기준.

## 시험 준비

1. 아두이노에 `firmware/fake_arduino/fake_arduino.ino` 업로드
2. 아두이노 IDE의 시리얼 모니터는 **닫는다** (COM 포트를 LabVIEW와 동시에 못 씀)
3. VISA Configure Serial Port: 115200 baud, 8 data bits, no parity, 1 stop bit, **Enable Termination Char = False** (0x0A가 데이터로 올 수 있음)

## subVI 1: `packet_send.vi`

- 입력: VISA resource, CMD(U8), D1(U8), D2(U8)
- 체크섬 = `(CMD + D1 + D2)`를 U8로 더하기 (U8 덧셈은 자동으로 하위 8비트만 남음)
- Build Array로 `[0x02, CMD, D1, D2, CHK, 0x03]` 만들기 → Byte Array To String → VISA Write
- 확인: `CMD=0x02, D1=0, D2=90` 보내면 `02 84 02 00 86 03`(동작완료)이 돌아와야 함

## subVI 2: `packet_receive.vi`

- 출력: 성공(Bool), CMD, D1, D2
- 순서
  1. `Bytes at Port` > 0이면 1바이트씩 읽어서 `0x02`가 나올 때까지 버리기
  2. 나머지 5바이트 읽기 (timeout 짧게, 예 100ms)
  3. 5번째 바이트(ETX) = `0x03`인지, 4번째(CHK) = CMD+D1+D2인지 검사
  4. 맞으면 성공=True + CMD/D1/D2 출력, 틀리면 성공=False (버리고 다음 호출에서 다시 STX 찾기)
- 16비트 값(거리, 배터리, 피크): `D1 × 256 + D2` (Join Numbers 사용 가능)

## 상태머신 뼈대: `main.vi`

- While Loop + Case Structure + Shift Register(상태, Enum: 대기 0 / 트리거감지 1 / 반응 2 / 탐색이동 3 / 회피 4 / 복귀 5)
- 매 반복: `packet_receive.vi` 호출 → 받은 CMD에 따라 처리
  - `0x81` 수신 + 현재 상태가 대기 → 트리거감지로
  - `0x82` → 거리 표시 / `0x83` → 배터리 표시 (10mV 단위 → ÷100 해서 V)
- 하트비트: 별도 While Loop에서 250ms마다 `packet_send.vi(0x07, 0, 0)`
  - 확인: 하트비트 루프를 멈추면 1초 뒤 `02 86 01 00 87 03`(통신두절)이 와야 함

## 1단계 완료 기준

- [ ] 가짜 아두이노의 박수(10초마다)로 대기 → 트리거감지 전환
- [ ] 거리·배터리 값이 화면에 계속 바뀜
- [ ] 하트비트 루프를 멈추면 0x86(통신두절) 수신
