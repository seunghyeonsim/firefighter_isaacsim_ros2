# 🚒 Firefighter Isaac Sim ROS2 Node

Isaac Sim을 사용한 소방로봇 시뮬레이션과 ROS2 노드를 통한 제어 시스템입니다.

## 📋 요구사항

- Ubuntu 22.04
- Isaac Sim 4.5
- ROS2 Humble
- Python 3.8+

## 🚀 빠른 시작

### 1. 시뮬레이션 실행

```bash
# 터미널 1에서 시뮬레이션 실행
./run_firefighter_sim.sh
```

### 2. 자동 테스트 실행

```bash
# 터미널 2에서 자동 테스트 실행
./test_firefighter_commands.sh
```

### 3. 수동 테스트

```bash
# 터미널 2에서 수동 테스트 명령어 로드
source manual_test_commands.sh

# 사용 예시
flipper_up 3.0                    # 모든 플리퍼 올리기
flipper_individual 2.0 -1.5 1.0 -0.5  # 개별 플리퍼 제어
drive_forward 0.5                 # 직진
turn_left 1.0                     # 좌회전
```

## 📡 ROS2 토픽

### 발행 토픽 (Publishers)
- `firefighter/status` (String): 시뮬레이션 상태
- `firefighter/joint_states` (JointState): 모든 조인트의 위치와 속도

### 구독 토픽 (Subscribers)
- `firefighter/cmd_vel` (Twist): 차동 구동 제어
- `firefighter/flipper_command` (FlipperCommand): 플리퍼 제어

## 🎮 제어 명령어

### 플리퍼 제어 (FlipperCommand)

#### 모든 플리퍼 정지
```bash
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'stop', fl_velocity: 0.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
```

#### 모든 플리퍼 동일 속도로 제어
```bash
# 모든 플리퍼를 3.0 rad/s로 올리기
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'all', fl_velocity: 3.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"

# 모든 플리퍼를 -2.0 rad/s로 내리기
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'all', fl_velocity: -2.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
```

#### 개별 플리퍼 제어
```bash
# 개별 플리퍼 제어
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'individual', fl_velocity: 3.0, fr_velocity: -2.0, rl_velocity: 1.5, rr_velocity: -1.5}"
```

### 주행 제어 (cmd_vel)

#### 직진
```bash
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 후진
```bash
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 좌회전
```bash
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

#### 우회전
```bash
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

#### 정지
```bash
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 📊 모니터링

### 토픽 모니터링
```bash
# 플리퍼 명령 모니터링
ros2 topic echo /firefighter/flipper_command

# 조인트 상태 모니터링
ros2 topic echo /firefighter/joint_states

# 상태 모니터링
ros2 topic echo /firefighter/status
```

### 토픽 정보 확인
```bash
# 토픽 목록
ros2 topic list | grep firefighter

# 토픽 정보
ros2 topic info /firefighter/flipper_command
ros2 topic info /firefighter/cmd_vel
```

## 🔧 커스텀 메시지

### FlipperCommand 구조
```
float64 fl_velocity  # Front Left flipper velocity
float64 fr_velocity  # Front Right flipper velocity  
float64 rl_velocity  # Rear Left flipper velocity
float64 rr_velocity  # Rear Right flipper velocity
string command_type  # "individual", "all", "stop"
```

### command_type 설명
- `"individual"`: 각 플리퍼를 개별적으로 제어
- `"all"`: 모든 플리퍼를 동일한 속도로 제어 (fl_velocity 값 사용)
- `"stop"`: 모든 플리퍼 정지

## 🏗️ 빌드

```bash
# 전체 워크스페이스 빌드
colcon build

# 특정 패키지만 빌드
colcon build --packages-select firefighter_description
colcon build --packages-select firefighter_interfaces
```

## 📁 파일 구조

```
firefighter_ws/
├── src/
│   ├── firefighter_description/          # 메인 패키지
│   │   ├── scripts/
│   │   │   ├── firefighter_isaacsim.py  # ROS2 노드
│   │   │   └── firefighter_articulation.py
│   │   ├── urdf/                        # 로봇 모델
│   │   └── usd/                         # Isaac Sim 씬
│   └── firefighter_interfaces/          # 커스텀 메시지
│       └── msg/
│           └── FlipperCommand.msg
├── run_firefighter_sim.sh               # 실행 스크립트
├── test_firefighter_commands.sh         # 자동 테스트
└── manual_test_commands.sh              # 수동 테스트
```

## 🚨 문제 해결

### Isaac Sim 초기화 실패
- `firefighter_scene.usd` 파일이 `usd/` 디렉토리에 있는지 확인
- Isaac Sim 경로가 올바른지 확인

### ROS2 메시지 인식 안됨
- 워크스페이스 소스 설정: `source install/setup.bash`
- `firefighter_interfaces` 패키지가 빌드되었는지 확인

### 플리퍼가 움직이지 않음
- URDF 파일의 joint limit 설정 확인
- Isaac Sim 시뮬레이션이 정상적으로 실행되고 있는지 확인

## 📞 지원

문제가 발생하면 다음을 확인하세요:
1. Isaac Sim 로그 메시지
2. ROS2 노드 로그 메시지
3. 토픽 연결 상태
4. 파일 경로 및 권한 