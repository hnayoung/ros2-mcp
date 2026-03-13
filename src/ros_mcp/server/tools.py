"""
G1 Humanoid Robot MCP Tools

MuJoCo `view_model.py` 브리지 기준 저수준 G1 제어용 MCP 도구 모음.

지원:
- `/lowcmd`
- `/lowstate`
- `/inspire/left/cmd`
- `/inspire/right/cmd`
- `/lf/inspire/left/state`
- `/lf/inspire/right/state`
- 29개 ROS2 관절
- 양손 6축 제어
- IMU 상태 조회

미지원:
- `/sportmodestate`
- `/api/sport/request`
- `/arm_sdk`
"""

from .ros2_mcp_server import mcp, get_robot


# ============================================================
# 1. 상태 조회 (Read)
# ============================================================

@mcp.tool(description="[G1] 전체 관절 상태 조회 (위치, 속도, 토크)")
def get_joint_states() -> dict:
    """
    `view_model.py`가 `/lowstate`에 publish 하는 29개 관절 상태를 조회합니다.

    Returns:
        positions: 관절 위치 (rad)
        velocities: 관절 속도 (rad/s)
        efforts: 관절 토크 (Nm)

    토픽: /lowstate
    """
    return get_robot().get_joint_states()


@mcp.tool(description="[G1] IMU 센서 데이터 조회")
def get_imu_state() -> dict:
    """
    `view_model.py`가 `/lowstate`에 넣는 IMU 상태를 조회합니다.

    Returns:
        quaternion: 쿼터니언 (w, x, y, z)
        rpy: 오일러 각도 (roll, pitch, yaw) (rad)
        gyroscope: 각속도 (rad/s)
        accelerometer: 가속도 (m/s^2)

    참고:
        free-base 모델이면 베이스 자세 기반 값이 들어갑니다.
        fixed-base 모델이면 0 값이 반환됩니다.

    토픽: /lowstate
    """
    return get_robot().get_imu_data()


@mcp.tool(description="[G1] Inspire hand 상태 조회")
def get_hand_states() -> dict:
    """
    MuJoCo Inspire hand 상태를 조회합니다.

    Returns:
        left_hand: 왼손 6축 상태
        right_hand: 오른손 6축 상태
    """
    return get_robot().get_hand_states()

# ============================================================
# 2. 저수준 관절 제어 (via /lowcmd)
# ============================================================

@mcp.tool(description="[G1] 단일 관절 명령 전송")
def send_joint_command(joint_id: int, q: float = 0.0, dq: float = 0.0,
                       tau: float = 0.0, kp: float = 50.0, kd: float = 1.0,
                       duration: float = 0.3) -> dict:
    """
    단일 관절 목표를 전송하되, 나머지 관절은 기본 자세를 유지하도록 전신 포즈로 제어합니다.

    Args:
        joint_id: ROS2 관절 인덱스 (0-28)
        q: 목표 위치 (rad)
        dq: 목표 속도 (rad/s)
        tau: 피드포워드 토크 (Nm)
        kp: 위치 게인
        kd: 속도 게인
        duration: 목표 자세로 보간 이동 시간 (초). 0이면 즉시 적용

    `view_model.py` 적용 공식:
        실제토크 = tau + kp*(q-현재위치) + kd*(dq-현재속도)

    토픽: /lowcmd
    """
    return get_robot().send_joint_command(joint_id, q, dq, tau, kp, kd, duration)


@mcp.tool(description="[G1] 전체 관절 위치 명령 전송")
def send_joint_positions(positions: list, kp: float = 50.0, kd: float = 1.0, duration: float = 1.0) -> dict:
    """
    `view_model.py`가 매핑한 29개 ROS2 관절 위치를 동시에 제어합니다.

    Args:
        positions: 29개 관절의 목표 위치 (rad)
        kp: 위치 게인
        kd: 속도 게인
        duration: 현재 MuJoCo 브리지에서는 사용하지 않음

    토픽: /lowcmd
    """
    return get_robot().send_joint_positions(positions, kp, kd, duration)


@mcp.tool(description="[G1] Inspire hand 명령 전송")
def send_hand_command(hand: str, positions: list, kp: float = 1.5, kd: float = 0.1) -> dict:
    """
    한쪽 Inspire hand의 독립 6축을 제어합니다.

    Args:
        hand: "left" 또는 "right"
        positions: 6개 관절 위치 (rad)
            [thumb_yaw, thumb_pitch, index, middle, ring, pinky]
        kp: 위치 게인
        kd: 속도 게인
    """
    return get_robot().send_hand_command(hand, positions, kp, kd)



# ============================================================
# 3. 설정/유틸리티
# ============================================================
@mcp.tool(description="[G1] 관절 정보 조회")
def get_joint_info() -> dict:
    """
    MuJoCo 브리지에서 사용하는 ROS2 관절 이름과 인덱스 매핑을 반환합니다.

    Returns:
        관절 이름, 인덱스, 현재 위치 정보
    """
    return get_robot().get_joint_info()


@mcp.tool(description="[G1] 관절 ID 매핑 조회")
def get_joint_id_map() -> dict:
    """
    관절 이름과 `send_joint_command`에 넣을 `joint_id` 매핑만 간단히 반환합니다.

    Returns:
        joint_ids: `{관절이름: joint_id}`
        groups: 다리/허리/팔 관절 인덱스 범위
    """
    info = get_robot().get_joint_info()
    if not info.get("success"):
        return info

    return {
        "success": True,
        "body_joint_ids": {
            joint_name: joint_info["id"]
            for joint_name, joint_info in info["joints"].items()
        },
        "hand_joint_ids": {
            hand_name: {
                joint_name: joint_info["id"]
                for joint_name, joint_info in hand_info.items()
            }
            for hand_name, hand_info in info.get("hand_joints", {}).items()
        },
        "groups": info["groups"],
    }


@mcp.tool(description="[G1] 사용 가능한 명령 목록")
def get_available_commands() -> dict:
    """
    G1 로봇에서 사용 가능한 모든 MCP 명령 목록을 반환합니다.

    Returns:
        low_level: 저수준 관절 제어 명령
        status: 저수준 상태 조회 명령
    """
    return {
        "success": True,
        "backend": "mujoco_view_model_lowlevel",
        "supported_topics": ["/lowcmd", "/lowstate"],
        "supported_hand_topics": [
            "/inspire/left/cmd",
            "/inspire/right/cmd",
            "/lf/inspire/left/state",
            "/lf/inspire/right/state",
        ],
        "unsupported_topics": ["/sportmodestate", "/api/sport/request", "/arm_sdk"],
        "notes": [
            "MuJoCo bridge는 29개 ROS2 관절만 제어합니다.",
            "Inspire hand는 손당 6개 독립축만 제어합니다.",
            "IMU 값은 free-base 모델일 때만 동적으로 채워집니다.",
        ],
        "low_level_commands": {
            "send_joint_command": "단일 관절 명령",
            "send_joint_positions": "전체 관절 위치",
            "send_hand_command": "한쪽 손 6축 명령"
        },
        "status_commands": {
            "get_joint_states": "관절 상태 조회",
            "get_hand_states": "손 상태 조회",
            "get_imu_state": "IMU 데이터 조회",
            "get_joint_info": "관절 이름/인덱스 정보",
            "get_joint_id_map": "관절 이름 -> joint_id 매핑"
        },
        "topic_info": {
            "/lowcmd": "저수준 관절 제어 (Publish)",
            "/lowstate": "관절 상태 (Subscribe)",
            "/inspire/left/cmd": "왼손 명령 (Publish)",
            "/inspire/right/cmd": "오른손 명령 (Publish)",
            "/lf/inspire/left/state": "왼손 상태 (Subscribe)",
            "/lf/inspire/right/state": "오른손 상태 (Subscribe)"
        }
    }
