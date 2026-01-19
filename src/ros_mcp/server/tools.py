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


# ============== G1 휴머노이드 전용 도구 ==============

@mcp.tool(description="[G1] 로봇 일어서기")
def stand_up() -> dict:
    """G1 로봇을 일어서게 합니다. 다른 동작 전에 먼저 실행해야 합니다."""
    return adapter.stand_up()


@mcp.tool(description="[G1] 로봇 앉기")
def sit_down() -> dict:
    """G1 로봇을 앉게 합니다."""
    return adapter.sit_down()


@mcp.tool(description="[G1] 걷기 명령")
def walk(direction: str = "forward", speed: float = 0.5, duration: float = 2.0) -> dict:
    """
    G1 로봇을 걷게 합니다.

    Args:
        direction: 이동 방향 (forward, backward, left, right)
        speed: 속도 (m/s), 기본값 0.5
        duration: 지속 시간 (초), 기본값 2.0
    """
    return adapter.walk(direction, speed, duration)


@mcp.tool(description="[G1] 제자리 회전")
def turn(angle: float = 90.0, direction: str = "left") -> dict:
    """
    G1 로봇을 제자리에서 회전시킵니다.

    Args:
        angle: 회전 각도 (도), 기본값 90
        direction: 회전 방향 (left, right)
    """
    return adapter.turn(angle, direction)


@mcp.tool(description="[G1] 목표 위치로 이동")
def walk_to(x: float, y: float) -> dict:
    """
    G1 로봇을 목표 좌표로 걸어서 이동시킵니다.

    Args:
        x: 목표 x 좌표 (m)
        y: 목표 y 좌표 (m)
    """
    return adapter.walk_to(x, y)


@mcp.tool(description="[G1] 보행 모드 설정")
def set_gait_mode(mode: int) -> dict:
    """
    G1 로봇의 보행 모드를 설정합니다.

    Args:
        mode: 0=서있기, 1=걷기, 2=달리기, 3=계단오르기
    """
    return adapter.set_gait_mode(mode)


@mcp.tool(description="[G1] 관절 상태 조회")
def get_joint_states() -> dict:
    """G1 로봇의 모든 관절 상태(위치, 속도, 토크)를 조회합니다."""
    return adapter.get_joint_states()


@mcp.tool(description="[G1] IMU 센서 데이터 조회")
def get_imu_data() -> dict:
    """G1 로봇의 IMU(관성 측정 장치) 데이터를 조회합니다."""
    return adapter.get_imu_data()


@mcp.tool(description="[G1] 로봇 상태 요약")
def get_robot_status() -> dict:
    """G1 로봇의 전체 상태(위치, 자세, 센서 등)를 요약합니다."""
    return adapter.get_robot_status()


@mcp.tool(description="[G1] 손 흔들기")
def wave_hand(hand: str = "right") -> dict:
    """
    G1 로봇이 손을 흔듭니다.

    Args:
        hand: 흔들 손 (left, right)
    """
    return adapter.wave_hand(hand)


@mcp.tool(description="[G1] 인사하기 (허리 숙이기)")
def bow() -> dict:
    """G1 로봇이 허리를 숙여 인사합니다."""
    return adapter.bow()


@mcp.tool(description="[G1] 댐핑 모드 (비상 안전)")
def set_damping_mode() -> dict:
    """G1 로봇을 댐핑 모드(안전 모드)로 전환합니다. 비상 시 사용합니다."""
    return adapter.set_damping_mode()


# ============== G1 저수준 명령 (Low-level Commands) ==============

@mcp.tool(description="[G1] 단일 관절 위치 제어")
def set_joint_position(joint_id: int, position: float, kp: float = 50.0, kd: float = 3.5) -> dict:
    """
    단일 관절의 위치를 제어합니다.

    Args:
        joint_id: 관절 인덱스 (0-28)
        position: 목표 위치 (rad)
        kp: 위치 게인 (기본값 50.0)
        kd: 속도 게인 (기본값 3.5)
    """
    return adapter.set_joint_position(joint_id, position, kp, kd)


@mcp.tool(description="[G1] 모든 관절 위치 제어")
def set_joint_positions(positions: list, kp: float = 50.0, kd: float = 3.5, duration: float = 1.0) -> dict:
    """
    모든 관절(29개)의 위치를 동시에 제어합니다.

    Args:
        positions: 29개 관절의 목표 위치 리스트 (rad)
        kp: 위치 게인 (기본값 50.0)
        kd: 속도 게인 (기본값 3.5)
        duration: 이동 시간 (초, 기본값 1.0)
    """
    return adapter.set_joint_positions(positions, kp, kd, duration)


@mcp.tool(description="[G1] 단일 관절 토크 제어")
def set_joint_torque(joint_id: int, torque: float) -> dict:
    """
    단일 관절에 토크를 직접 적용합니다 (위치 제어 없이).

    Args:
        joint_id: 관절 인덱스 (0-28)
        torque: 토크 값 (Nm)
    """
    return adapter.set_joint_torque(joint_id, torque)


@mcp.tool(description="[G1] 저수준 명령 직접 전송")
def send_low_cmd(joint_cmds: list) -> dict:
    """
    저수준 모터 명령을 직접 전송합니다. PD 제어 파라미터를 완전히 제어할 수 있습니다.

    Args:
        joint_cmds: 관절 명령 리스트. 각 항목은 dict:
            {
                "id": 관절 인덱스 (0-28),
                "q": 목표 위치 (rad),
                "kp": 위치 게인,
                "dq": 목표 속도 (rad/s),
                "kd": 속도 게인,
                "tau": 토크 (Nm)
            }
    """
    return adapter.send_low_cmd(joint_cmds)


@mcp.tool(description="[G1] 관절 정보 조회")
def get_joint_info() -> dict:
    """G1 로봇의 관절 이름과 인덱스 매핑 정보를 반환합니다."""
    return adapter.get_joint_info()
