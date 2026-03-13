"""
ROS2 MCP Server

MCP(Model Context Protocol)를 통해 G1 로봇을 제어하는 서버
"""

import atexit
import os
import threading

from fastmcp import FastMCP
from ros_mcp.server.g1_robot import G1Robot


mcp = FastMCP("ros2-mcp-server")

robot_namespace = os.getenv("ROBOT_NAMESPACE", "")
robot = None
robot_lock = threading.Lock()


def get_robot() -> G1Robot:
    global robot
    with robot_lock:
        if robot is None or getattr(robot, "_shutdown", False):
            robot = G1Robot(robot_namespace=robot_namespace)
    return robot


def _shutdown_robot():
    global robot
    if robot is not None:
        robot.shutdown()
        robot = None


atexit.register(_shutdown_robot)


def main():
    from ros_mcp.server import tools  # noqa: F401

    # FastMCP tool 호출 시점이 아니라 서버 시작 시점에 ROS2를 올린다.
    # 노드 lifecycle을 단순화해서 stale RMW 상태를 피한다.
    get_robot()
    mcp.run(transport="stdio", show_banner=False)


if __name__ == "__main__":
    main()
