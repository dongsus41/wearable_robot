# 웨어러블 로봇 제어 시스템

이 저장소는 ROS2 기반 웨어러블 로봇 제어 시스템의 소스 코드를 포함하고 있습니다. CAN 통신을 통해 센서 데이터를 수집하고 액추에이터를 제어하는 패키지들을 제공합니다.

## 주요 기능

- CAN 통신을 통한 센서 데이터 수집 및 처리
- 변위 센서, IMU 센서, 온도 센서 데이터 처리
- 액추에이터 제어 및 온도 관리
- 허리 보조 제어 시스템

## 요구 사항

- Ubuntu 20.04/22.04
- ROS2 (Foxy/Humble/Iron)
- CAN 인터페이스 (하드웨어)
- 의존성: can-utils, ros2_socketcan

## 설치 방법

### 1. 워크스페이스 생성

```bash
mkdir -p ~/wearable_ws/src
cd ~/wearable_ws/src
```

### 2. 소스 코드 복제

```bash
git clone https://github.com/dongsus41/wearable_robot.git
```

### 3. 의존성 설치

```bash
# ROS2 의존성
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-std-srvs can-utils

# ros2_socketcan 패키지 설치 (필요한 경우)
cd ~/wearable_ws/src
git clone https://github.com/ros-drivers/ros2_socketcan.git
```

### 4. 빌드

```bash
cd ~/wearable_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```

### 5. 환경 설정

```bash
source ~/wearable_ws/install/setup.bash

# .bashrc에 추가하려면:
echo "source ~/wearable_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 사용 방법

### CAN 인터페이스 설정

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 berr-reporting on fd on restart-ms 100
sudo ifconfig can0 txqueuelen 1000

sudo ip link set can0 down
sudo ip link set can0 up

```

### 전체 시스템 실행

```bash
ros2 launch wearable_robot_bringup bringup_all.launch.py
```

### 개별 노드 실행

```bash
# 데이터 처리 노드만 실행
ros2 launch wearable_robot_data_processing data_processing.launch.py

# 특정 노드 실행
ros2 run wearable_robot_data_processing data_parser_node
```

## 프로젝트 구조

```
src/
├── wearable_robot_interfaces/    # 메시지 및 서비스 정의
├── wearable_robot_data_processing/  # 데이터 처리 노드
├── wearable_robot_control/       # 액추에이터 제어 노드
└── wearable_robot_bringup/       # 실행 설정 및 launch 파일
```

## 문제 해결

- **의존성 문제**: `rosdep install --from-paths src --ignore-src -r -y`를 실행하여 필요한 의존성을 자동으로 설치하세요.
- **CAN 통신 오류**: CAN 인터페이스가 올바르게 설정되었는지 확인하세요. `candump can0`으로 CAN 메시지를 확인할 수 있습니다.
- **빌드 오류**: 특정 패키지에 문제가 있다면 `colcon build --symlink-install --packages-select <패키지_이름>`으로 해당 패키지만 다시 빌드하세요.


