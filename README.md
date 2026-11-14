# robot-pet

디지털시스템설계 팀 프로젝트: 박수에 반응하는 펫로봇 (아두이노 Uno + LabVIEW 블루투스 연동)

| 폴더 | 내용 |
|---|---|
| `firmware/` | 아두이노 스케치 (`fake_arduino/`: LabVIEW 통신 시험용) |
| `labview/` | LabVIEW `.vi` 파일과 구현 가이드 |
| `docs/` | 통신 규칙 표 등 문서 |
| `progress/` | 단계별 진행 기록 (설명 + 스크린샷) |

- 통신 규칙: [docs/protocol.md](docs/protocol.md) (변경 시 HW·SW 둘 다 합의)
- HW 담당은 최신 코드를 받아 업로드만 한다.

## 진행 상황

노션 체크리스트 단계별로 나눠 기록한다. 각 폴더의 README에 항목별 설명·테스트·사진이 있다.

| 단계 | 내용 (SW) | 상태 |
|---|---|---|
| [1단계](progress/1단계/) | 가짜 아두이노, 패킷 송신·수신 subVI, 상태머신 뼈대, 박수 → 트리거감지, VISA 연결 | ✅ SW 완료 (통합 점검 ① 대기) |
| [2단계](progress/2단계/) | 펌웨어 receivePacket·handleCommand, 주기 작업, 하트비트 | 🚧 진행 중 (receivePacket ✅, handleCommand ✅, 주기 작업 ✅) |
