#!/bin/bash

# 수동 테스트용 명령어 모음
# Usage: source manual_test_commands.sh
# 
# 이 스크립트를 source로 실행하면 함수들을 사용할 수 있습니다.

echo "🔧 수동 테스트 명령어 로드됨"
echo "사용법: source manual_test_commands.sh"
echo ""

# 워크스페이스로 이동 및 환경 설정
cd /home/van/firefighter_ws
source install/setup.bash

# 플리퍼 제어 함수들
function flipper_stop() {
    echo "🛑 모든 플리퍼 정지"
    ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'stop', fl_velocity: 0.0, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
}

function flipper_up() {
    local vel=${1:-3.0}
    echo "⬆️ 모든 플리퍼 올리기 (속도: $vel)"
    ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'all', fl_velocity: $vel, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
}

function flipper_down() {
    local vel=${1:--3.0}
    echo "⬇️ 모든 플리퍼 내리기 (속도: $vel)"
    ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'all', fl_velocity: $vel, fr_velocity: 0.0, rl_velocity: 0.0, rr_velocity: 0.0}"
}

function flipper_individual() {
    local fl=${1:-0.0}
    local fr=${2:-0.0}
    local rl=${3:-0.0}
    local rr=${4:-0.0}
    echo "🎯 개별 플리퍼 제어 - FL:$fl, FR:$fr, RL:$rl, RR:$rr"
    ros2 topic pub /firefighter/flipper_command firefighter_interfaces/msg/FlipperCommand "{command_type: 'individual', fl_velocity: $fl, fr_velocity: $fr, rl_velocity: $rl, rr_velocity: $rr}"
}

# 차동 구동 함수들
function drive_forward() {
    local speed=${1:-0.3}
    echo "➡️ 직진 (속도: $speed m/s)"
    ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: $speed, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
}

function drive_backward() {
    local speed=${1:-0.3}
    echo "⬅️ 후진 (속도: $speed m/s)"
    ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -$speed, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
}

function turn_left() {
    local speed=${1:-1.0}
    echo "⬅️ 좌회전 (각속도: $speed rad/s)"
    ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $speed}}"
}

function turn_right() {
    local speed=${1:-1.0}
    echo "➡️ 우회전 (각속도: $speed rad/s)"
    ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -$speed}}"
}

function drive_stop() {
    echo "🛑 주행 정지"
    ros2 topic pub /firefighter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
}

# 토픽 모니터링 함수들
function monitor_flipper() {
    echo "📡 플리퍼 명령 모니터링 시작 (Ctrl+C로 종료)"
    ros2 topic echo /firefighter/flipper_command
}

function monitor_joints() {
    echo "📡 조인트 상태 모니터링 시작 (Ctrl+C로 종료)"
    ros2 topic echo /firefighter/joint_states
}

function monitor_status() {
    echo "📡 상태 모니터링 시작 (Ctrl+C로 종료)"
    ros2 topic echo /firefighter/status
}

# 도움말 함수
function show_help() {
    echo ""
    echo "🚒 Firefighter 수동 테스트 명령어"
    echo "================================================"
    echo "플리퍼 제어:"
    echo "  flipper_stop                    - 모든 플리퍼 정지"
    echo "  flipper_up [속도]               - 모든 플리퍼 올리기 (기본: 3.0)"
    echo "  flipper_down [속도]             - 모든 플리퍼 내리기 (기본: -3.0)"
    echo "  flipper_individual fl fr rl rr  - 개별 플리퍼 제어"
    echo ""
    echo "주행 제어:"
    echo "  drive_forward [속도]            - 직진 (기본: 0.3 m/s)"
    echo "  drive_backward [속도]           - 후진 (기본: 0.3 m/s)"
    echo "  turn_left [각속도]              - 좌회전 (기본: 1.0 rad/s)"
    echo "  turn_right [각속도]             - 우회전 (기본: 1.0 rad/s)"
    echo "  drive_stop                      - 주행 정지"
    echo ""
    echo "모니터링:"
    echo "  monitor_flipper                 - 플리퍼 명령 모니터링"
    echo "  monitor_joints                  - 조인트 상태 모니터링"
    echo "  monitor_status                  - 상태 모니터링"
    echo ""
    echo "예시:"
    echo "  flipper_up 2.5                  - 모든 플리퍼를 2.5 rad/s로 올리기"
    echo "  flipper_individual 3.0 -2.0 1.5 -1.5  - 개별 플리퍼 제어"
    echo "  drive_forward 0.5               - 0.5 m/s로 직진"
    echo "================================================"
}

echo "사용 가능한 함수들:"
show_help 