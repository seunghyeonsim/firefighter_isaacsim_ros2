#!/bin/bash

# Firefighter Isaac Sim ROS2 Node 실행 스크립트
# Usage: ./run_firefighter_sim.sh

echo "🚒 Firefighter Isaac Sim ROS2 Node 실행 중..."
echo "================================================"

# 워크스페이스로 이동
cd /home/van/firefighter_ws

# 소스 환경 설정
echo "📦 ROS2 환경 설정 중..."
source install/setup.bash

# 노드 실행
echo "🚀 firefighter_isaacsim 노드 실행 중..."
echo "Ctrl+C로 종료할 수 있습니다."
echo "================================================"

ros2 run firefighter_description firefighter_isaacsim.py 