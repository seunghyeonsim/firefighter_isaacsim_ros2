#!/bin/bash

# Firefighter 명령어 테스트 스크립트
# Usage: ./test_firefighter_commands.sh
# 
# 이 스크립트를 실행하기 전에 다른 터미널에서 run_firefighter_sim.sh를 실행해야 합니다.

echo "🧪 Firefighter 명령어 테스트 스크립트"
echo "================================================"
echo "주의: 이 스크립트를 실행하기 전에 다른 터미널에서"
echo "run_firefighter_sim.sh를 실행하여 시뮬레이션을 시작하세요!"
echo "================================================"

# 워크스페이스로 이동
cd /home/van/firefighter_ws

# 소스 환경 설정
source install/setup.bash

# 토픽 목록 확인
echo ""
echo "📡 사용 가능한 토픽 목록:"
ros2 topic list | grep firefighter

echo ""
echo "⏳ 5초 후 테스트를 시작합니다..."
sleep 5

echo ""
echo "🎮 테스트 1: 모든 플리퍼 정지"
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'stop', fl_velocity: 0.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
sleep 2

echo ""
echo "🎮 테스트 2: 모든 플리퍼 올리기 (3.0 rad/s)"
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'all', fl_velocity: 3.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
sleep 3

echo ""
echo "🎮 테스트 3: 모든 플리퍼 내리기 (-2.0 rad/s)"
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'all', fl_velocity: -2.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
sleep 3

echo ""
echo "🎮 테스트 4: 모든 플리퍼 정지"
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'stop', fl_velocity: 0.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
sleep 2

echo ""
echo "🎮 테스트 5: 개별 플리퍼 제어"
echo "  - 왼쪽 앞: 2.0, 오른쪽 앞: -1.5, 왼쪽 뒤: 1.0, 오른쪽 뒤: -0.5"
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'individual', fl_velocity: 2.0, fr_velocity: -1.5, rl_velocity: 1.0, rr_velocity: -0.5}"
sleep 4

echo ""
echo "🎮 테스트 6: 모든 플리퍼 정지"
ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'stop', fl_velocity: 0.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
sleep 2

echo ""
echo "🎮 테스트 7: 차동 구동 테스트 - 직진"
echo "  - 선속도: 0.3 m/s, 각속도: 0.0 rad/s"
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 3

echo ""
echo "🎮 테스트 8: 차동 구동 테스트 - 정지"
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

echo ""
echo "🎮 테스트 9: 차동 구동 테스트 - 좌회전"
echo "  - 선속도: 0.0 m/s, 각속도: 1.0 rad/s"
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
sleep 3

echo ""
echo "🎮 테스트 10: 차동 구동 테스트 - 정지"
ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

echo ""
echo "🎉 모든 테스트 완료!"
echo "================================================"
echo "토픽 모니터링 명령어:"
echo "  - 플리퍼 명령: ros2 topic echo /firefighter/flipper_command"
echo "  - 조인트 상태: ros2 topic echo /firefighter/joint_states"
echo "  - 상태: ros2 topic echo /firefighter/status"
echo "================================================" 