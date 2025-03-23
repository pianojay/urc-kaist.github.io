---
title: "매니퓰레이터 소프트웨어"
weight: 1
draft: false
---

# 매니퓰레이터 소프트웨어

이 ROS2 작업공간은 로봇팔 제어를 위한 여러 패키지 및 설정 파일을 포함합니다.

## 로봇팔 동작 실행 방법

1. **로봇팔 초기 위치 설정(필수)**  
   - 전원을 켜기 전 로봇팔을 초기 위치로 이동시켜야 함.  
   - 초기 위치 확인 방법: 로봇팔 연결 없이 MoveIt을 실행했을 때 RViz에서 표시되는 자세가 0도 위치.

   ```bash
   source install/setup.bash
   ros2 launch rover_arm_moveit_config servo.launch.py
   ```

2. **Xbox 컨트롤러 연결**

3. **CAN 통신 Bitrate 설정**

   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```

4. **Dynamixel (J6) latency timer 설정**

   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   ```

5. **로봇팔 실행**

   ```bash
   ros2 launch rover_arm_moveit_config servo.launch.py
   ```

## 주의사항 및 개선 필요 사항

- **J1 발산 위험**: 첫 실행 시, CAN 연결을 끊고 실행 (모터 전원 연결은 가능)
- **J4, J5 벨트 제거 후 실행 권장**  
  - 최초 실행 시 진동 발생 가능  
  - 초기 위치가 잘못 설정되면 J4, J5가 360도 회전할 수 있음
- **동작 중 CAN 연결이 끊어지면 로봇 발산 위험 있음**

## 패키지 요약

| **패키지명**              | **설명**                                             |
|---------------------------|-------------------------------------------------------|
| rover_arm_description     | 로봇 URDF 모델 포함                                   |
| rover_arm                 | 원통형 URDF 모델 (현재 미사용)                         |
| rover_arm_moveit_config   | MoveIt 설정 및 실행 패키지                             |
| rover_arm_moveit_cpp      | MoveIt 관련 C++ 노드 포함                              |

## 패키지 상세 설명

### 1. rover_arm_description

로봇의 URDF 모델을 포함하는 패키지로, Fusion 360을 사용하여 생성되었습니다. STL 파일과 충돌 모델(Collision body)이 포함되어 있습니다.

원래 사용되던  
``fusion2urdf-ros2 <https://github.com/dheena2k2/fusion2urdf-ros2>``  
는 동작하지 않아, 대신  
``Robotizer-be의 fork <https://github.com/Robotizer-be/fusion2urdf-ros2>``  
를 사용합니다.

**설정 전 필수 작업:**

- **STL 파일명 정리**: `(1)` 등의 문구 제거  
  예: ``link1 (1).stl`` → ``link1.stl``
- **Launch 파일 경로 수정**: ``display.launch.py``에서 경로 확인 및 수정
- **Xacro 매크로 호출 추가**: ``rover_arm_macro.urdf.xacro`` 파일 끝부분에 아래 코드 추가

   ```xml
   <xacro:rover_arm prefix=""/>
   ```

- **base_link 미표시 문제 해결**: ``base_link`` 정의 수정

   ```xml
   <link name="${prefix}base_link">
     <visual>
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <geometry>
         <mesh filename="package://rover_arm_description/meshes/base_link.stl" 
               scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="rover_arm_silver"/>
     </visual>
     <collision>
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <geometry>
         <mesh filename="package://rover_arm_description/meshes/base_link.stl" 
               scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   ```

**주요 명령어:**

- URDF 모델 확인

   ```bash
   ros2 launch rover_arm_description display.launch.py
   ```

### 2. rover_arm (현재 미사용)

원통형 URDF 모델을 포함하는 패키지입니다. (현재 실행되지 않음)

**주요 명령어:**

- URDF 확인

   ```bash
   ros2 launch rover_arm display.launch.py
   ```

- Joint 별 각도 설정 슬라이더 실행

   ```bash
   ros2 run joint_state_publisher_gui joint_state_publisher_gui
   ```

### 3. rover_arm_moveit_config

MoveIt을 이용한 로봇팔 제어 및 설정 패키지입니다.

**주요 명령어:**

- MoveIt 로봇팔 동작

   ```bash
   ros2 launch rover_arm_moveit_config servo.launch.py
   ```

- MoveIt 설정 실행

   ```bash
   source install/setup.bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```

- MoveIt 및 RViz 실행

   ```bash
   ros2 launch rover_arm_moveit_config demo.launch.py
   ```

**알려진 버그:**

- 새로 설정 후 ``rover_arm_moveit_config/config/joint_limits.yaml`` 파일에서 limit 값을 소수점 형식으로 수정해야 함  
  예시: ``1`` → ``1.0``

### 4. rover_arm_moveit_cpp

MoveIt 관련 C++ 노드를 포함하는 패키지입니다.

**주요 명령어:**

   ```bash
   ros2 run rover_arm_moveit_cpp hello_moveit
   ```

