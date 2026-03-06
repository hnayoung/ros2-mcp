"""
ROS2 MCP Server

MCP(Model Context Protocol)를 통해 G1 로봇을 제어하는 서버

환경 변수:
    ROBOT_NAMESPACE: 로봇 네임스페이스 (기본값: "")
"""

import os
from fastmcp import FastMCP

from .g1_robot import G1Robot


# MCP 서버 초기화
mcp = FastMCP("ros2-mcp-server")

# G1 로봇 인스턴스 생성
robot_namespace = os.getenv("ROBOT_NAMESPACE", "")
robot = G1Robot(robot_namespace=robot_namespace)
