"""
G1 Humanoid Robot MCP Tools

실제 G1 로봇 제어를 위한 MCP 도구 모음
- High-Level: /api/sport/request (FSM 기반)
- Low-Level: /lowcmd (관절 직접 제어)
- Arm: /arm_sdk (팔 전용 제어)
"""

from .ros2_mcp_server import mcp, get_robot


# ============================================================
# 1. 상태 조회 (Read)
# ============================================================

@mcp.tool(description="[G1] 전체 관절 상태 조회 (위치, 속도, 토크)")
def get_joint_states() -> dict:
    """
    G1 로봇의 29개 관절 상태를 조회합니다.

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
    G1 로봇의 IMU 센서 데이터를 조회합니다.

    Returns:
        quaternion: 쿼터니언 (w, x, y, z)
        rpy: 오일러 각도 (roll, pitch, yaw) (rad)
        gyroscope: 각속도 (rad/s)
        accelerometer: 가속도 (m/s^2)

    토픽: /lowstate
    """
    return get_robot().get_imu_data()


@mcp.tool(description="[G1] 로봇 위치/속도 조회")
def get_robot_position() -> dict:
    """
    G1 로봇의 현재 위치와 속도를 조회합니다.

    Returns:
        position: [x, y, z] (m)
        velocity: [vx, vy, vz] (m/s)
        yaw_speed: 요 각속도 (rad/s)

    토픽: /sportmodestate
    """
    return get_robot().get_robot_position()


@mcp.tool(description="[G1] 현재 스포츠 모드 상태 조회")
def get_sport_mode() -> dict:
    """
    G1 로봇의 현재 스포츠 모드 상태를 조회합니다.

    Returns:
        mode: 현재 모드
        gait_type: 걸음 타입
        progress: 동작 진행률
        foot_raise_height: 발 들어올림 높이

    토픽: /sportmodestate
    """
    return get_robot().get_sport_mode()


@mcp.tool(description="[G1] 현재 FSM 상태 ID 조회")
def get_fsm_id() -> dict:
    """
    G1 로봇의 현재 FSM(Finite State Machine) 상태 ID를 조회합니다.

    FSM ID:
        0: ZeroTorque (토크 제로)
        1: Damp (댐핑)
        2: Squat (쪼그려 앉기)
        3: Sit (앉기)
        4: StandUp (일어서기)
        500: Start (시작)

    토픽: /api/sport/request → /api/sport/response
    """
    return get_robot().get_fsm_id()


@mcp.tool(description="[G1] 배터리 상태 조회")
def get_battery_state() -> dict:
    """
    G1 로봇의 배터리 상태를 조회합니다.

    Returns:
        voltage: 전압 (V)
        current: 전류 (A)
        soc: 충전 상태 (%)
        temperature: 온도 (°C)
    """
    return get_robot().get_battery_state()


@mcp.tool(description="[G1] 로봇 전체 상태 요약")
def get_robot_status() -> dict:
    """G1 로봇의 전체 상태를 요약하여 반환합니다."""
    return get_robot().get_robot_status()


# ============================================================
# 2. 고수준 동작 (High-Level via /api/sport/request)
# ============================================================

@mcp.tool(description="[G1] 로봇 일어서기 (FSM 4)")
def stand_up() -> dict:
    """
    G1 로봇을 일어서게 합니다.
    다른 동작 전에 먼저 실행해야 합니다.

    API ID: 7101 (SetFsmId)
    FSM ID: 4
    """
    return get_robot().stand_up()


@mcp.tool(description="[G1] 로봇 앉기 (FSM 3)")
def sit_down() -> dict:
    """
    G1 로봇을 앉게 합니다.

    API ID: 7101 (SetFsmId)
    FSM ID: 3
    """
    return get_robot().sit_down()


@mcp.tool(description="[G1] 쪼그려 앉기 (FSM 2)")
def squat() -> dict:
    """
    G1 로봇을 쪼그려 앉게 합니다.

    API ID: 7101 (SetFsmId)
    FSM ID: 2
    """
    return get_robot().squat()


@mcp.tool(description="[G1] 댐핑 모드 - 긴급 정지 (FSM 1)")
def damp() -> dict:
    """
    G1 로봇을 댐핑 모드로 전환합니다.
    긴급 상황에서 안전하게 정지시킬 때 사용합니다.

    API ID: 7101 (SetFsmId)
    FSM ID: 1
    """
    return get_robot().damp()


@mcp.tool(description="[G1] 토크 제로 모드 (FSM 0)")
def zero_torque() -> dict:
    """
    G1 로봇의 모든 모터 토크를 0으로 설정합니다.
    로봇이 늘어지게 됩니다. 주의해서 사용하세요.

    API ID: 7101 (SetFsmId)
    FSM ID: 0
    """
    return get_robot().zero_torque()


@mcp.tool(description="[G1] 시작 모드 (FSM 500)")
def start() -> dict:
    """
    G1 로봇을 시작 모드로 전환합니다.

    API ID: 7101 (SetFsmId)
    FSM ID: 500
    """
    return get_robot().start()


@mcp.tool(description="[G1] 이동 명령")
def move(vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0, duration: float = 1.0) -> dict:
    """
    G1 로봇을 지정된 속도로 이동시킵니다.

    Args:
        vx: 전진 속도 (m/s), 양수=전진, 음수=후진
        vy: 횡방향 속도 (m/s), 양수=왼쪽, 음수=오른쪽
        vyaw: 회전 속도 (rad/s), 양수=반시계, 음수=시계
        duration: 지속 시간 (초)

    API ID: 7105 (SetVelocity)
    """
    return get_robot().move(vx, vy, vyaw, duration)


@mcp.tool(description="[G1] 이동 정지")
def stop_move() -> dict:
    """
    G1 로봇의 이동을 즉시 정지합니다.

    API ID: 7105 (SetVelocity with 0, 0, 0)
    """
    return get_robot().stop_move()


@mcp.tool(description="[G1] 서있는 높이 설정")
def set_stand_height(height: float) -> dict:
    """
    G1 로봇의 서있는 높이를 설정합니다.

    Args:
        height: 높이 (m), 0.0 ~ 1.0 범위

    API ID: 7104 (SetStandHeight)
    """
    return get_robot().set_stand_height(height)


@mcp.tool(description="[G1] 발 스윙 높이 설정")
def set_swing_height(height: float) -> dict:
    """
    G1 로봇의 걸을 때 발 스윙 높이를 설정합니다.

    Args:
        height: 높이 (m)

    API ID: 7103 (SetSwingHeight)
    """
    return get_robot().set_swing_height(height)


@mcp.tool(description="[G1] 속도 모드 설정")
def set_speed_mode(mode: int) -> dict:
    """
    G1 로봇의 속도 모드를 설정합니다.

    Args:
        mode: 속도 모드 (0, 1, 2)

    API ID: 7107 (SetSpeedMode)
    """
    return get_robot().set_speed_mode(mode)


# ============================================================
# 3. 제스처/퍼포먼스 (High-Level)
# ============================================================

@mcp.tool(description="[G1] 손 흔들기")
def wave_hand() -> dict:
    """
    G1 로봇이 손을 흔듭니다.

    API ID: 7106 (SetTaskId)
    Task ID: 0
    """
    return get_robot().wave_hand()


@mcp.tool(description="[G1] 돌면서 손 흔들기")
def wave_hand_with_turn() -> dict:
    """
    G1 로봇이 돌면서 손을 흔듭니다.

    API ID: 7106 (SetTaskId)
    Task ID: 1
    """
    return get_robot().wave_hand_with_turn()


@mcp.tool(description="[G1] 악수 시작")
def shake_hand_start() -> dict:
    """
    G1 로봇이 악수 자세를 취합니다.
    shake_hand_end()로 종료해야 합니다.

    API ID: 7106 (SetTaskId)
    Task ID: 2
    """
    return get_robot().shake_hand_start()


@mcp.tool(description="[G1] 악수 종료")
def shake_hand_end() -> dict:
    """
    G1 로봇의 악수 동작을 종료합니다.

    API ID: 7106 (SetTaskId)
    Task ID: 3
    """
    return get_robot().shake_hand_end()


# ============================================================
# 4. 팔 제어 (via /arm_sdk)
# ============================================================

@mcp.tool(description="[G1] 왼팔 위치 제어")
def set_left_arm_position(positions: list, duration: float = 1.0) -> dict:
    """
    G1 로봇의 왼팔 관절 위치를 제어합니다.

    Args:
        positions: 7개 관절 위치 (rad)
            [shoulder_pitch, shoulder_roll, shoulder_yaw,
             elbow, wrist_roll, wrist_pitch, wrist_yaw]
        duration: 이동 시간 (초)

    토픽: /arm_sdk
    """
    return get_robot().set_left_arm_position(positions, duration)


@mcp.tool(description="[G1] 오른팔 위치 제어")
def set_right_arm_position(positions: list, duration: float = 1.0) -> dict:
    """
    G1 로봇의 오른팔 관절 위치를 제어합니다.

    Args:
        positions: 7개 관절 위치 (rad)
            [shoulder_pitch, shoulder_roll, shoulder_yaw,
             elbow, wrist_roll, wrist_pitch, wrist_yaw]
        duration: 이동 시간 (초)

    토픽: /arm_sdk
    """
    return get_robot().set_right_arm_position(positions, duration)


@mcp.tool(description="[G1] 양팔 동시 위치 제어")
def set_both_arms_position(left_positions: list, right_positions: list, duration: float = 1.0) -> dict:
    """
    G1 로봇의 양팔 관절 위치를 동시에 제어합니다.

    Args:
        left_positions: 왼팔 7개 관절 위치 (rad)
        right_positions: 오른팔 7개 관절 위치 (rad)
        duration: 이동 시간 (초)

    토픽: /arm_sdk
    """
    return get_robot().set_both_arms_position(left_positions, right_positions, duration)


@mcp.tool(description="[G1] 허리 위치 제어")
def set_waist_position(yaw: float = 0.0, roll: float = 0.0, pitch: float = 0.0, duration: float = 1.0) -> dict:
    """
    G1 로봇의 허리 관절 위치를 제어합니다.

    Args:
        yaw: 허리 요 (rad)
        roll: 허리 롤 (rad) - 일부 모델에서 잠김
        pitch: 허리 피치 (rad) - 일부 모델에서 잠김
        duration: 이동 시간 (초)

    토픽: /arm_sdk
    """
    return get_robot().set_waist_position(yaw, roll, pitch, duration)


# ============================================================
# 5. 저수준 관절 제어 (via /lowcmd)
# ============================================================

@mcp.tool(description="[G1] 단일 관절 명령 전송")
def send_joint_command(joint_id: int, q: float = 0.0, dq: float = 0.0,
                       tau: float = 0.0, kp: float = 50.0, kd: float = 1.0) -> dict:
    """
    단일 관절에 저수준 명령을 전송합니다.

    Args:
        joint_id: 관절 인덱스 (0-28)
        q: 목표 위치 (rad)
        dq: 목표 속도 (rad/s)
        tau: 피드포워드 토크 (Nm)
        kp: 위치 게인
        kd: 속도 게인

    제어 공식: 실제토크 = tau + kp*(q-현재위치) + kd*(dq-현재속도)

    토픽: /lowcmd
    """
    return get_robot().send_joint_command(joint_id, q, dq, tau, kp, kd)


@mcp.tool(description="[G1] 전체 관절 위치 명령 전송")
def send_joint_positions(positions: list, kp: float = 50.0, kd: float = 1.0, duration: float = 1.0) -> dict:
    """
    모든 관절(29개)의 위치를 동시에 제어합니다.

    Args:
        positions: 29개 관절의 목표 위치 (rad)
        kp: 위치 게인
        kd: 속도 게인
        duration: 이동 시간 (초)

    토픽: /lowcmd
    """
    return get_robot().send_joint_positions(positions, kp, kd, duration)


@mcp.tool(description="[G1] 다리 관절 명령 전송")
def send_leg_command(leg: str, positions: list, kp: float = 100.0, kd: float = 2.0) -> dict:
    """
    한쪽 다리의 6개 관절을 제어합니다.

    Args:
        leg: "left" 또는 "right"
        positions: 6개 관절 위치 (rad)
            [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]
        kp: 위치 게인
        kd: 속도 게인

    토픽: /lowcmd
    """
    return get_robot().send_leg_command(leg, positions, kp, kd)


# ============================================================
# 6. 설정/유틸리티
# ============================================================

@mcp.tool(description="[G1] 균형 모드 설정")
def set_balance_mode(mode: int) -> dict:
    """
    G1 로봇의 균형 모드를 설정합니다.

    Args:
        mode: 0=균형 서기, 1=연속 걸음 모드

    API ID: 7102 (SetBalanceMode)
    """
    return get_robot().set_balance_mode(mode)


@mcp.tool(description="[G1] 연속 걸음 모드 활성화/비활성화")
def enable_continuous_gait(enable: bool) -> dict:
    """
    G1 로봇의 연속 걸음 모드를 활성화/비활성화합니다.

    Args:
        enable: True=활성화, False=비활성화
    """
    return get_robot().enable_continuous_gait(enable)


@mcp.tool(description="[G1] 관절 정보 조회")
def get_joint_info() -> dict:
    """
    G1 로봇의 관절 이름과 인덱스 매핑 정보를 반환합니다.

    Returns:
        관절 이름, 인덱스, 현재 위치 정보
    """
    return get_robot().get_joint_info()


@mcp.tool(description="[G1] 사용 가능한 명령 목록")
def get_available_commands() -> dict:
    """
    G1 로봇에서 사용 가능한 모든 MCP 명령 목록을 반환합니다.

    Returns:
        high_level: 고수준 명령 (FSM)
        arm_control: 팔 제어 명령
        low_level: 저수준 관절 제어 명령
        status: 상태 조회 명령
    """
    return {
        "success": True,
        "high_level_commands": {
            "stand_up": "일어서기 (FSM 4)",
            "sit_down": "앉기 (FSM 3)",
            "squat": "쪼그려 앉기 (FSM 2)",
            "damp": "댐핑/긴급정지 (FSM 1)",
            "zero_torque": "토크 제로 (FSM 0)",
            "start": "시작 (FSM 500)",
            "move": "이동 (vx, vy, vyaw)",
            "stop_move": "이동 정지",
            "set_stand_height": "서있는 높이 설정",
            "set_swing_height": "발 스윙 높이 설정",
            "set_speed_mode": "속도 모드 설정"
        },
        "gesture_commands": {
            "wave_hand": "손 흔들기",
            "wave_hand_with_turn": "돌면서 손 흔들기",
            "shake_hand_start": "악수 시작",
            "shake_hand_end": "악수 종료"
        },
        "arm_commands": {
            "set_left_arm_position": "왼팔 위치 제어",
            "set_right_arm_position": "오른팔 위치 제어",
            "set_both_arms_position": "양팔 동시 제어",
            "set_waist_position": "허리 위치 제어"
        },
        "low_level_commands": {
            "send_joint_command": "단일 관절 명령",
            "send_joint_positions": "전체 관절 위치",
            "send_leg_command": "다리 관절 명령"
        },
        "status_commands": {
            "get_joint_states": "관절 상태 조회",
            "get_imu_state": "IMU 데이터 조회",
            "get_robot_position": "로봇 위치/속도 조회",
            "get_sport_mode": "스포츠 모드 상태",
            "get_fsm_id": "FSM 상태 ID",
            "get_battery_state": "배터리 상태",
            "get_robot_status": "전체 상태 요약"
        },
        "topic_info": {
            "/lowcmd": "저수준 관절 제어 (Publish)",
            "/lowstate": "관절 상태 (Subscribe)",
            "/api/sport/request": "고수준 명령 (Publish)",
            "/api/sport/response": "명령 응답 (Subscribe)",
            "/sportmodestate": "스포츠 모드 상태 (Subscribe)",
            "/arm_sdk": "팔 제어 (Publish)"
        }
    }
