---
title: "STM32 Drive Motor Controller"
weight: 1
draft: false
---

# STM32 Drive Motor Controller


## Basic Settings

- **baudrate**: 921600  
- **CRC**: CRC32, IO reflected, mask=0x000...

> **Note:**  
> This directory contains materials related to the firmware implementation of the STM32-based Velocity Control function.  
> The system is designed to receive various commands via UART communication and control motor speed and braking based on PID control.
> For more details about the Velocity Control for STM32, refer to the Notion page:  
>
> [Velocity Control STM32 기능 및 통신](https://www.notion.so/mrurc/Velocity-control-stm32-60fa8861a17d4f1194d312a2be62e80e)

---

## Main Features

### 1. UART Communication

- Communicates with external systems via UART to process various command modes:
  - **INIT_STATE**: System initialization
  - **SET_SPEED**: Set motor speed
  - **CURRENT_SPEED**: Transmit current motor speed
  - **SET_BRAKE**: Set brake
  - **SET_PID**: Set PID parameters
  - **CURRENT_PID**: Transmit current PID parameters

### 2. CRC-based Packet Verification

- Data received via UART is verified for integrity using CRC32 (with input and output reflected).
- In the event of a CRC verification failure, an error message is sent to handle the issue.

### 3. PID Control

- Performs PID control for each wheel to improve the accuracy of speed control.
- The parameters **Kp**, **Ki**, **Kd**, and **Ka** can be set and read.

### 4. Brake and Speed Control

- Sets the speed for a specific motor ID or for all motors.
- Brake control is implemented to ensure safe stopping operations.

---

## Code Components

The following are the main code files:

### 1. `UART_handler.cpp`

- Handles UART data reception and packet parsing.
- Includes processing of various command modes and data transmission.

### 2. `Drive_system.cpp`

- Implements PID-based speed and brake control.
- Contains the logic for measuring and controlling motor speed.

### 3. `real_main.cpp`

- The main entry point, which includes system initialization and the main loop.
- Configures HAL (TIM, UART, etc.) and handles timer interrupts.

---

## Usage

### 1. Build and Upload the Code

- Open the project in STM32CubeIDE, build the project, and upload it to the target board.

### 2. UART Communication Settings

- Use UART2 for communication with external systems.
- Default buffer sizes:
  - RX: 24 bytes
  - TX: 24 bytes

### 3. Using Key Commands

- System initialization: `INIT_STATE` mode
- Speed setting: `SET_SPEED` mode
- Brake setting: `SET_BRAKE` mode
- PID parameter setting: `SET_PID` mode

---

## Cautions

### 1. CRC Error Handling

- If a CRC error occurs, the error code should be retried along with the data.

### 2. Motor Settings

- Motor control is based on motor ID; if the ID is `0`, the command is applied to all motors.

### 3. DMA Initialization

- If the UART DMA buffer is not initialized, manual initialization is required to prevent reception issues.

---

## Inquiries and Revisions

For further inquiries about the code, please contact the project manager or report issues through the team's issue tracker.
