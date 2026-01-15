import os
from fastmcp import FastMCP

from .adapters import TurtlesimAdapter

# MCP 서버 초기화
mcp = FastMCP("ros2-mcp-server")

# 어떤 로봇을 쓸 지 결정 (환경 변수 또는 기본값)
robot_type = os.getenv("ROBOT_TYPE", "turtlesim")

# adapter 생성
if robot_type == "turtlesim":
    adapter = TurtlesimAdapter()
else:
    raise ValueError(f"해당하는 로봇 타입이 없습니다: {robot_type}")
