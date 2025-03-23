---
title: "Stm32 Drive Motor Controller"
weight: 1
draft: false
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---
# STM32 Drive Motor Controller

For more details about the Velocity Control for STM32, refer to the Notion page:  
[Velocity Control STM32 기능 및 통신](https://www.notion.so/mrurc/Velocity-control-stm32-60fa8861a17d4f1194d312a2be62e80e)

## 기본 설정

- **baudrate**: 921600  
- **CRC**: CRC32, IO reflected, mask=0x000...

> **Note:**  
> 이 디렉토리는 STM32 기반의 Velocity Control 기능을 구현한 펌웨어와 관련된 자료를 포함합니다.  
> 본 시스템은 UART 통신을 통해 다양한 명령을 수신하고, PID 제어를 기반으로 모터의 속도 및 브레이크를 제어하도록 설계되었습니다.

---

## 주요 기능

### 1. UART 기반 통신

- 외부 시스템과 UART를 통해 데이터를 주고받으며, 다양한 명령어 모드 처리:
  - **INIT_STATE**: 시스템 초기화
  - **SET_SPEED**: 모터 속도 설정
  - **CURRENT_SPEED**: 현재 모터 속도 전송
  - **SET_BRAKE**: 브레이크 설정
  - **SET_PID**: PID 파라미터 설정
  - **CURRENT_PID**: 현재 PID 파라미터 전송

### 2. CRC 기반 패킷 검증

- UART로 수신된 데이터는 CRC32를 통해 무결성 검증을 수행합니다 (input, output reflected).
- CRC 검증 실패 시 오류 메시지를 전송하여 문제를 처리합니다.

### 3. PID 제어

- 각 바퀴에 대해 PID 제어를 수행하여 속도 제어의 정확성을 향상시킵니다.
- **Kp**, **Ki**, **Kd**, **Ka** 파라미터를 설정 및 읽기 가능합니다.

### 4. 브레이크 및 속도 제어

- 특정 ID 또는 모든 모터의 속도 설정.
- 브레이크 제어를 통해 안전한 정지 동작 수행.

---

## 코드 구성 요소

다음은 주요 코드 파일입니다:

### 1. `UART_handler.cpp`

- UART 데이터 수신 및 패킷 파싱 처리.
- 다양한 모드의 명령어 처리 및 데이터 전송 기능 포함.

### 2. `Drive_system.cpp`

- PID 기반 속도 및 브레이크 제어 구현.
- 모터 속도 측정 및 제어 로직 포함.

### 3. `real_main.cpp`

- 메인 진입점으로 시스템 초기화 및 루프 동작을 포함.
- HAL (TIM, UART 등) 설정 및 타이머 인터럽트 처리.

---

## 사용 방법

### 1. 코드 빌드 및 업로드

- STM32CubeIDE에서 프로젝트를 열고 빌드 후 타겟 보드에 업로드합니다.

### 2. UART 통신 설정

- UART2를 사용하여 외부 시스템과 통신합니다.
- 기본 버퍼 크기:
  - RX: 24 바이트
  - TX: 24 바이트

### 3. 주요 명령 사용

- 시스템 초기화: `INIT_STATE` 모드
- 속도 설정: `SET_SPEED` 모드
- 브레이크 설정: `SET_BRAKE` 모드
- PID 파라미터 설정: `SET_PID` 모드

---

## 주의 사항

### 1. CRC 오류 처리

- CRC 오류 발생 시, 오류 코드와 함께 재시도해야 합니다.

### 2. 모터 설정

- ID를 기반으로 모터 제어를 수행하며, ID가 `0`일 경우 모든 모터에 명령이 적용됩니다.

### 3. DMA 초기화

- UART DMA 버퍼가 초기화되지 않은 경우, 수신 문제를 방지하기 위해 수동 초기화가 필요합니다.

---

## 문의 및 수정

코드에 대한 추가 문의 사항은 프로젝트 담당자에게 연락하거나, 팀의 이슈 트래커를 통해 보고하시기 바랍니다.
