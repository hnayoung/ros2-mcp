"""
MCP 서버 엔트리포인트
python -m ros_mcp.server 로 실행
"""
from ros_mcp.server.ros2_mcp_server import mcp
from ros_mcp.server import tools  # noqa: F401 - 도구 등록을 위해 import

if __name__ == "__main__":
    mcp.run(transport="stdio", show_banner=False)
