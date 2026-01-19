#!/bin/bash
# G1 시뮬레이션 + MCP 서버 실행 스크립트

echo "=== G1 시뮬레이션 환경 시작 ==="

# ROS 2 환경 소스
source /opt/ros/humble/setup.bash

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}[1/3] MuJoCo G1 시뮬레이터 시작...${NC}"
echo "      (별도 터미널에서 실행하세요)"
echo ""
echo "      cd ~/unitree_ws/src/unitree_mujoco/simulate_python"
echo "      python3 ./unitree_mujoco.py"
echo ""

read -p "시뮬레이터가 실행되면 Enter를 누르세요..."

echo -e "${GREEN}[2/3] ROS 2 브릿지 시작...${NC}"
cd /home/na0/mcp_server
python3 scripts/g1_ros2_bridge.py &
BRIDGE_PID=$!
echo "      Bridge PID: $BRIDGE_PID"
sleep 2

echo -e "${GREEN}[3/3] MCP 서버 시작 (ROBOT_TYPE=g1)...${NC}"
export ROBOT_TYPE=g1
export ROS_LOG_DIR=/tmp
python3 -m ros_mcp.server

# 종료 시 정리
echo -e "${YELLOW}종료 중...${NC}"
kill $BRIDGE_PID 2>/dev/null
echo -e "${GREEN}완료${NC}"
