---
title: "STM32 모터 속도 제어"
weight: 1
draft: false
---
# STM32 모터 속도 제어

## 기본 통신 설정

- **Baudrate**: 921600
- **CRC**: CRC32 (Input/Output Reflected)
- **UART**: UART2 사용

## UART 통신 프로토콜

UART를 통해 다음과 같은 명령어 모드를 지원합니다:

- **INIT_STATE**: 시스템 초기화
- **SET_SPEED**: 모터 속도 설정
- **CURRENT_SPEED**: 현재 속도 전송
- **SET_BRAKE**: 브레이크 설정
- **SET_PID**: PID 파라미터 설정
- **CURRENT_PID**: 현재 PID 파라미터 전송

## CRC 검증 방법

- UART 데이터의 무결성 검증을 위한 CRC32 (input/output reflected) 사용
- CRC 오류 발생 시 오류 메시지를 반환하여 재시도 유도

## 코드 구성 요소

주요 파일:

- **UART_handler.cpp**  
  - UART 데이터 수신 및 패킷 파싱  
  - 명령어 모드 처리 및 데이터 전송

- **Drive_system.cpp**  
  - PID 기반 모터 속도 및 브레이크 제어  
  - 속도 측정 및 제어 로직 포함

- **real_main.cpp**  
  - 메인 진입점, 시스템 초기화 및 주 루프  
  - HAL 설정 (TIM, UART 등) 및 인터럽트 처리

## 사용 방법

1. **STM32CubeIDE에서 빌드 및 업로드**
2. **UART 통신 설정**
   - RX/TX 기본 버퍼 크기: 24 바이트
3. **명령 사용 예시**:
   - 시스템 초기화: ``INIT_STATE``
   - 속도 설정: ``SET_SPEED``
   - 브레이크 설정: ``SET_BRAKE``
   - PID 설정: ``SET_PID``

## 주의 사항

- CRC 오류 발생 시 오류 코드 반환
- ID가 0이면 모든 모터에 적용
- DMA 초기화 미진 시 수신 오류 가능성

## 문의 및 수정

추가 문의 사항은 프로젝트 담당자 또는 팀 이슈 트래커를 활용 바랍니다.
