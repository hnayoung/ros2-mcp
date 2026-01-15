from .ros2_mcp_server import mcp, adapter


# ============== ROS 2 시스템 조회 도구 ==============

@mcp.tool(description="사용 가능한 모든 ROS 2 토픽 목록 조회")
def get_topics() -> dict:
    """ROS 2에서 사용 가능한 토픽을 가져옵니다."""
    return adapter.get_topics()


@mcp.tool(description="특정 토픽의 메시지 타입 조회")
def get_topic_type(topic: str) -> dict:
    """특정 토픽의 메시지 타입을 가져옵니다."""
    return adapter.get_topic_type(topic)


@mcp.tool(description="특정 토픽의 publisher 개수 조회")
def get_topic_publishers_count(topic: str) -> dict:
    """특정 토픽의 publisher 개수를 가져옵니다."""
    return adapter.get_topic_publishers_count(topic)


@mcp.tool(description="특정 토픽의 subscriber 개수 조회")
def get_topic_subscribers_count(topic: str) -> dict:
    """특정 토픽의 subscriber 개수를 가져옵니다."""
    return adapter.get_topic_subscribers_count(topic)


@mcp.tool(description="사용 가능한 모든 ROS 2 서비스 목록 조회")
def get_services() -> dict:
    """사용 가능한 모든 ROS 서비스 목록을 가져옵니다."""
    return adapter.get_services()


@mcp.tool(description="특정 서비스의 타입 조회")
def get_service_type(service: str) -> dict:
    """특정 서비스의 타입을 가져옵니다."""
    return adapter.get_service_type(service)


@mcp.tool(description="현재 실행 중인 모든 ROS 2 노드 목록 조회")
def get_nodes() -> dict:
    """현재 실행 중인 모든 ROS 노드 목록을 가져옵니다."""
    return adapter.get_nodes()


@mcp.tool(description="특정 노드의 모든 파라미터 목록 조회")
def get_node_parameters(node_name: str) -> dict:
    """특정 노드의 모든 파라미터 목록을 가져옵니다."""
    return adapter.get_node_parameters(node_name)


@mcp.tool(description="ROS 2 시스템 정보 조회")
def get_ros_info() -> dict:
    """배포판 및 버전을 포함한 ROS 2 시스템 정보를 가져옵니다."""
    return adapter.get_ros_info()


@mcp.tool(description="ROS 2 연결 상태 확인")
def check_ros_connection() -> dict:
    """ROS 2가 초기화되고 실행 중인지 확인합니다."""
    return adapter.check_ros_connection()


# ============== 로봇 이동 도구 ==============

@mcp.tool(description="로봇을 앞으로 직진 이동")
def move_forward(speed: float = 1.0, duration: float = 1.0) -> dict:
    """
    로봇을 직선으로 앞으로 이동시킵니다.

    Args:
        speed: 선속도 (m/s), 기본값 1.0
        duration: 이동 지속 시간 (초), 기본값 1.0
    """
    return adapter.move_forward(speed, duration)


@mcp.tool(description="로봇을 뒤로 후진 이동")
def move_backward(speed: float = 1.0, duration: float = 1.0) -> dict:
    """
    로봇을 직선으로 뒤로 이동시킵니다.

    Args:
        speed: 선속도 (m/s), 기본값 1.0
        duration: 이동 지속 시간 (초), 기본값 1.0
    """
    return adapter.move_backward(speed, duration)


@mcp.tool(description="로봇을 제자리에서 회전")
def rotate(angle: float = 1.57, angular_speed: float = 1.0, direction: str = "left") -> dict:
    """
    로봇을 제자리에서 회전시킵니다.

    Args:
        angle: 회전할 각도 (라디안), 기본값 π/2 (90도)
        angular_speed: 회전 속도 (rad/s), 기본값 1.0
        direction: 회전 방향 ("left" 또는 "right"), 기본값 "left"
    """
    return adapter.rotate(angle, angular_speed, direction)


@mcp.tool(description="로봇을 즉시 정지")
def stop() -> dict:
    """0 속도를 전송하여 로봇을 정지시킵니다."""
    return adapter.stop()


@mcp.tool(description="로봇의 현재 위치/방향 조회")
def get_pose() -> dict:
    """로봇의 현재 위치와 방향을 조회합니다."""
    return adapter.get_pose()
