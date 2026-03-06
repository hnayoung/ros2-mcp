"""
G1 Humanoid Robot Controller

실제 G1 로봇 제어
- High-Level: /api/sport/request, /api/sport/response (FSM 기반)
- Low-Level: /lowcmd, /lowstate (관절 직접 제어)
- Arm: /arm_sdk (팔 전용 제어)
- Status: /sportmodestate (로봇 상태)
"""

import json
import time
import threading
import numpy as np
from typing import Optional, List, Dict, Any

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

# Unitree ROS2 메시지 타입
try:
    from unitree_hg.msg import LowCmd, LowState
    from unitree_go.msg import SportModeState, WirelessController
    from unitree_api.msg import Request, Response
    UNITREE_ROS2_AVAILABLE = True
except ImportError:
    UNITREE_ROS2_AVAILABLE = False
    print("[WARN] unitree messages not found.")
    print("       Source: source ~/unitree_ws/src/unitree_ros2/setup_local.sh")


# ============================================================
# G1 API ID 상수 (g1_loco_client.hpp 참조)
# ============================================================

# Get API IDs
ROBOT_API_ID_LOCO_GET_FSM_ID = 7001
ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002
ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003
ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004
ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005
ROBOT_API_ID_LOCO_GET_PHASE = 7006

# Set API IDs
ROBOT_API_ID_LOCO_SET_FSM_ID = 7101
ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102
ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103
ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104
ROBOT_API_ID_LOCO_SET_VELOCITY = 7105
ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106
ROBOT_API_ID_LOCO_SET_SPEED_MODE = 7107

# FSM IDs
FSM_ID_ZERO_TORQUE = 0
FSM_ID_DAMP = 1
FSM_ID_SQUAT = 2
FSM_ID_SIT = 3
FSM_ID_STAND_UP = 4
FSM_ID_START = 500

# Task IDs (for arm gestures)
TASK_ID_WAVE_HAND = 0
TASK_ID_WAVE_HAND_WITH_TURN = 1
TASK_ID_SHAKE_HAND_START = 2
TASK_ID_SHAKE_HAND_END = 3


class G1Robot:
    """
    Unitree G1 휴머노이드 로봇 제어

    토픽:
    - /lowcmd: 저수준 모터 명령 (Publish)
    - /lowstate: 저수준 모터 상태 (Subscribe)
    - /arm_sdk: 팔 제어 명령 (Publish)
    """

    ROBOT_NAME = 'g1'
    NUM_MOTORS = 29

    # 관절 인덱스
    class JointIndex:
        # 왼쪽 다리 (0-5)
        LEFT_HIP_PITCH = 0
        LEFT_HIP_ROLL = 1
        LEFT_HIP_YAW = 2
        LEFT_KNEE = 3
        LEFT_ANKLE_PITCH = 4
        LEFT_ANKLE_ROLL = 5
        # 오른쪽 다리 (6-11)
        RIGHT_HIP_PITCH = 6
        RIGHT_HIP_ROLL = 7
        RIGHT_HIP_YAW = 8
        RIGHT_KNEE = 9
        RIGHT_ANKLE_PITCH = 10
        RIGHT_ANKLE_ROLL = 11
        # 허리 (12-14)
        WAIST_YAW = 12
        WAIST_ROLL = 13
        WAIST_PITCH = 14
        # 왼팔 (15-21)
        LEFT_SHOULDER_PITCH = 15
        LEFT_SHOULDER_ROLL = 16
        LEFT_SHOULDER_YAW = 17
        LEFT_ELBOW = 18
        LEFT_WRIST_ROLL = 19
        LEFT_WRIST_PITCH = 20
        LEFT_WRIST_YAW = 21
        # 오른팔 (22-28)
        RIGHT_SHOULDER_PITCH = 22
        RIGHT_SHOULDER_ROLL = 23
        RIGHT_SHOULDER_YAW = 24
        RIGHT_ELBOW = 25
        RIGHT_WRIST_ROLL = 26
        RIGHT_WRIST_PITCH = 27
        RIGHT_WRIST_YAW = 28

    # 관절 이름
    JOINT_NAMES = [
        "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
        "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
        "right_knee", "right_ankle_pitch", "right_ankle_roll",
        "waist_yaw", "waist_roll", "waist_pitch",
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
        "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"
    ]

    # 기본 PD 게인
    DEFAULT_KP = {
        'leg': 100.0,
        'waist': 60.0,
        'arm': 50.0
    }
    DEFAULT_KD = {
        'leg': 2.0,
        'waist': 1.5,
        'arm': 1.0
    }

    def __init__(self, robot_namespace: str = ""):
        """G1 로봇 초기화"""
        self.namespace = robot_namespace

        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node('g1_mcp_robot')
        self.cmd_vel_topic = "/cmd_vel"
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            self.cmd_vel_topic,
            QoSProfile(depth=10)
        )

        # 상태 변수
        self.low_state: Optional[LowState] = None
        self.sport_mode_state: Optional[SportModeState] = None
        self.api_response: Optional[Response] = None
        self.response_received = threading.Event()

        self.is_standing = False
        self.continuous_gait_enabled = False
        self.current_fsm_id = -1

        # ROS2 초기화
        if UNITREE_ROS2_AVAILABLE:
            self._init_ros2_interfaces()
        else:
            print("[G1Robot] WARNING: unitree messages not available!")

    def _init_ros2_interfaces(self):
        """ROS2 인터페이스 초기화"""

        # QoS 설정
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ===== Publishers =====
        # 저수준 모터 명령
        self.lowcmd_pub = self.node.create_publisher(
            LowCmd, "/lowcmd", qos_best_effort
        )

        # 고수준 API 요청
        self.api_request_pub = self.node.create_publisher(
            Request, "/api/sport/request", qos_reliable
        )

        # 팔 제어 (arm_sdk)
        self.arm_sdk_pub = self.node.create_publisher(
            LowCmd, "/arm_sdk", qos_reliable
        )

        # ===== Subscribers =====
        # 저수준 모터 상태
        self.lowstate_sub = self.node.create_subscription(
            LowState, "/lowstate", self._lowstate_callback, qos_best_effort
        )

        # 고수준 API 응답
        self.api_response_sub = self.node.create_subscription(
            Response, "/api/sport/response", self._api_response_callback, qos_reliable
        )

        # 스포츠 모드 상태
        self.sportmode_sub = self.node.create_subscription(
            SportModeState, "/sportmodestate", self._sportmode_callback, qos_best_effort
        )

        print("[G1Robot] ROS2 interfaces initialized")
        print(f"  Publishers: /lowcmd, /api/sport/request, /arm_sdk")
        print(f"  Subscribers: /lowstate, /api/sport/response, /sportmodestate")

    # ============================================================
    # 콜백 함수
    # ============================================================

    def _lowstate_callback(self, msg: LowState):
        """저수준 상태 콜백"""
        self.low_state = msg

    def _api_response_callback(self, msg: Response):
        """API 응답 콜백"""
        self.api_response = msg
        self.response_received.set()

    def _sportmode_callback(self, msg: SportModeState):
        """스포츠 모드 상태 콜백"""
        self.sport_mode_state = msg

    # ============================================================
    # 헬퍼 함수
    # ============================================================

    def _send_api_request(self, api_id: int, parameter: dict = None, wait_response: bool = True, timeout: float = 2.0) -> dict:
        """API 요청 전송"""
        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        req = Request()
        req.header.identity.api_id = api_id

        if parameter:
            req.parameter = json.dumps(parameter)
        else:
            req.parameter = ""

        self.response_received.clear()
        self.api_request_pub.publish(req)

        if wait_response:
            if self.response_received.wait(timeout):
                if self.api_response:
                    return {
                        "success": True,
                        "data": self.api_response.data if hasattr(self.api_response, 'data') else None
                    }
            return {"error": "Response timeout"}

        return {"success": True, "message": "Request sent (no response wait)"}

    def _create_low_cmd(self) -> LowCmd:
        """LowCmd 메시지 생성"""
        cmd = LowCmd()
        cmd.mode_pr = 0
        cmd.mode_machine = 0

        for i in range(35):
            cmd.motor_cmd[i].mode = 0
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].reserve = 0

        return cmd

    def _get_kp_kd(self, joint_id: int) -> tuple:
        """관절별 기본 KP, KD 값 반환"""
        if joint_id < 12:  # 다리
            return self.DEFAULT_KP['leg'], self.DEFAULT_KD['leg']
        elif joint_id < 15:  # 허리
            return self.DEFAULT_KP['waist'], self.DEFAULT_KD['waist']
        else:  # 팔
            return self.DEFAULT_KP['arm'], self.DEFAULT_KD['arm']

    # ============================================================
    # 1. 상태 조회 (Read)
    # ============================================================

    def _spin_once(self):
        """ROS 콜백 한 번 처리"""
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def get_joint_states(self) -> dict:
        """관절 상태 조회"""
        self._spin_once()

        if self.low_state:
            positions = []
            velocities = []
            efforts = []

            for i in range(self.NUM_MOTORS):
                positions.append(round(float(self.low_state.motor_state[i].q), 4))
                velocities.append(round(float(self.low_state.motor_state[i].dq), 4))
                efforts.append(round(float(self.low_state.motor_state[i].tau_est), 4))

            return {
                "success": True,
                "positions": positions,
                "velocities": velocities,
                "efforts": efforts,
                "joint_names": self.JOINT_NAMES
            }

        return {"error": "LowState not received", "hint": "Check /lowstate topic"}

    def get_imu_data(self) -> dict:
        """IMU 데이터 조회"""
        self._spin_once()

        if self.low_state:
            imu = self.low_state.imu_state
            return {
                "success": True,
                "quaternion": list(imu.quaternion),
                "rpy": list(imu.rpy),
                "gyroscope": list(imu.gyroscope),
                "accelerometer": list(imu.accelerometer)
            }

        return {"error": "LowState not received"}

    def get_robot_position(self) -> dict:
        """로봇 위치/속도 조회"""
        self._spin_once()

        if self.sport_mode_state:
            return {
                "success": True,
                "position": list(self.sport_mode_state.position),
                "velocity": list(self.sport_mode_state.velocity),
                "yaw_speed": float(self.sport_mode_state.yaw_speed),
                "body_height": float(self.sport_mode_state.body_height)
            }

        return {"error": "SportModeState not received", "hint": "Check /sportmodestate topic"}

    def get_sport_mode(self) -> dict:
        """스포츠 모드 상태 조회"""
        self._spin_once()

        if self.sport_mode_state:
            return {
                "success": True,
                "mode": int(self.sport_mode_state.mode),
                "gait_type": int(self.sport_mode_state.gait_type),
                "progress": float(self.sport_mode_state.progress),
                "foot_raise_height": float(self.sport_mode_state.foot_raise_height),
                "foot_force": list(self.sport_mode_state.foot_force)
            }

        return {"error": "SportModeState not received"}

    def get_fsm_id(self) -> dict:
        """FSM 상태 ID 조회"""
        result = self._send_api_request(ROBOT_API_ID_LOCO_GET_FSM_ID)

        if result.get("success"):
            try:
                data = json.loads(result.get("data", "{}"))
                self.current_fsm_id = data.get("data", -1)
                return {
                    "success": True,
                    "fsm_id": self.current_fsm_id,
                    "fsm_name": self._get_fsm_name(self.current_fsm_id)
                }
            except:
                pass

        return result

    def _get_fsm_name(self, fsm_id: int) -> str:
        """FSM ID를 이름으로 변환"""
        names = {
            0: "ZeroTorque",
            1: "Damp",
            2: "Squat",
            3: "Sit",
            4: "StandUp",
            500: "Start"
        }
        return names.get(fsm_id, f"Unknown({fsm_id})")

    def get_battery_state(self) -> dict:
        """배터리 상태 조회"""
        self._spin_once()

        # BMS 상태는 LowState에 포함될 수 있음 (로봇 모델에 따라 다름)
        if self.low_state:
            return {
                "success": True,
                "note": "Battery info may vary by robot model",
                "mode_machine": int(self.low_state.mode_machine)
            }

        return {"error": "LowState not received"}

    def get_robot_status(self) -> dict:
        """로봇 전체 상태 요약"""
        self._spin_once()

        status = {
            "success": True,
            "robot_name": self.ROBOT_NAME,
            "ros2_available": UNITREE_ROS2_AVAILABLE,
            "is_standing": self.is_standing,
            "continuous_gait": self.continuous_gait_enabled,
            "lowstate_received": self.low_state is not None,
            "sportmode_received": self.sport_mode_state is not None
        }

        if self.sport_mode_state:
            status["position"] = list(self.sport_mode_state.position)
            status["mode"] = int(self.sport_mode_state.mode)

        return status

    # ============================================================
    # 2. 고수준 동작 (High-Level via /api/sport/request)
    # ============================================================

    def _set_fsm_id(self, fsm_id: int) -> dict:
        """FSM ID 설정 (내부 함수)"""
        result = self._send_api_request(
            ROBOT_API_ID_LOCO_SET_FSM_ID,
            {"data": fsm_id},
            wait_response=True
        )

        if result.get("success"):
            self.current_fsm_id = fsm_id

        return result

    def stand_up(self) -> dict:
        """일어서기 (FSM 4)"""
        result = self._set_fsm_id(FSM_ID_STAND_UP)
        if result.get("success"):
            self.is_standing = True
            return {"success": True, "action": "stand_up", "fsm_id": FSM_ID_STAND_UP}
        return result

    def sit_down(self) -> dict:
        """앉기 (FSM 3)"""
        result = self._set_fsm_id(FSM_ID_SIT)
        if result.get("success"):
            self.is_standing = False
            return {"success": True, "action": "sit_down", "fsm_id": FSM_ID_SIT}
        return result

    def squat(self) -> dict:
        """쪼그려 앉기 (FSM 2)"""
        result = self._set_fsm_id(FSM_ID_SQUAT)
        if result.get("success"):
            return {"success": True, "action": "squat", "fsm_id": FSM_ID_SQUAT}
        return result

    def damp(self) -> dict:
        """댐핑 모드 - 긴급 정지 (FSM 1)"""
        result = self._set_fsm_id(FSM_ID_DAMP)
        if result.get("success"):
            self.is_standing = False
            return {"success": True, "action": "damp", "fsm_id": FSM_ID_DAMP}
        return result

    def zero_torque(self) -> dict:
        """토크 제로 모드 (FSM 0)"""
        result = self._set_fsm_id(FSM_ID_ZERO_TORQUE)
        if result.get("success"):
            self.is_standing = False
            return {"success": True, "action": "zero_torque", "fsm_id": FSM_ID_ZERO_TORQUE}
        return result

    def start(self) -> dict:
        """시작 모드 (FSM 500)"""
        result = self._set_fsm_id(FSM_ID_START)
        if result.get("success"):
            return {"success": True, "action": "start", "fsm_id": FSM_ID_START}
        return result

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0, duration: float = 1.0) -> dict:
        """이동 명령 (SetVelocity)"""
        result = self._send_api_request(
            ROBOT_API_ID_LOCO_SET_VELOCITY,
            {
                "velocity": [vx, vy, vyaw],
                "duration": duration
            },
            wait_response=False  # 이동 명령은 응답 대기 안 함
        )

        return {
            "success": True,
            "action": "move",
            "vx": vx,
            "vy": vy,
            "vyaw": vyaw,
            "duration": duration
        }

    def stop_move(self) -> dict:
        """이동 정지"""
        return self.move(0.0, 0.0, 0.0, 0.0)

    def set_stand_height(self, height: float) -> dict:
        """서있는 높이 설정"""
        result = self._send_api_request(
            ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
            {"data": height}
        )

        if result.get("success"):
            return {"success": True, "action": "set_stand_height", "height": height}
        return result

    def set_swing_height(self, height: float) -> dict:
        """발 스윙 높이 설정"""
        result = self._send_api_request(
            ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
            {"data": height}
        )

        if result.get("success"):
            return {"success": True, "action": "set_swing_height", "height": height}
        return result

    def set_speed_mode(self, mode: int) -> dict:
        """속도 모드 설정"""
        result = self._send_api_request(
            ROBOT_API_ID_LOCO_SET_SPEED_MODE,
            {"data": mode}
        )

        if result.get("success"):
            return {"success": True, "action": "set_speed_mode", "mode": mode}
        return result

    def set_balance_mode(self, mode: int) -> dict:
        """균형 모드 설정"""
        result = self._send_api_request(
            ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
            {"data": mode}
        )

        if result.get("success"):
            return {"success": True, "action": "set_balance_mode", "mode": mode}
        return result

    def enable_continuous_gait(self, enable: bool) -> dict:
        """연속 걸음 모드 활성화/비활성화"""
        mode = 1 if enable else 0
        result = self.set_balance_mode(mode)

        if result.get("success"):
            self.continuous_gait_enabled = enable
            return {"success": True, "action": "enable_continuous_gait", "enabled": enable}
        return result

    # ============================================================
    # 3. 제스처/퍼포먼스 (High-Level)
    # ============================================================

    def _set_task_id(self, task_id: int) -> dict:
        """Task ID 설정 (제스처용)"""
        return self._send_api_request(
            ROBOT_API_ID_LOCO_SET_ARM_TASK,
            {"data": task_id},
            wait_response=False
        )

    def wave_hand(self) -> dict:
        """손 흔들기"""
        result = self._set_task_id(TASK_ID_WAVE_HAND)
        return {"success": True, "action": "wave_hand", "task_id": TASK_ID_WAVE_HAND}

    def wave_hand_with_turn(self) -> dict:
        """돌면서 손 흔들기"""
        result = self._set_task_id(TASK_ID_WAVE_HAND_WITH_TURN)
        return {"success": True, "action": "wave_hand_with_turn", "task_id": TASK_ID_WAVE_HAND_WITH_TURN}

    def shake_hand_start(self) -> dict:
        """악수 시작"""
        result = self._set_task_id(TASK_ID_SHAKE_HAND_START)
        return {"success": True, "action": "shake_hand_start", "task_id": TASK_ID_SHAKE_HAND_START}

    def shake_hand_end(self) -> dict:
        """악수 종료"""
        result = self._set_task_id(TASK_ID_SHAKE_HAND_END)
        return {"success": True, "action": "shake_hand_end", "task_id": TASK_ID_SHAKE_HAND_END}

    # ============================================================
    # 4. 팔 제어 (via /arm_sdk)
    # ============================================================

    def _send_arm_cmd(self, joint_positions: dict, duration: float = 1.0) -> dict:
        """팔 명령 전송 (/arm_sdk)"""
        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        cmd = self._create_low_cmd()

        for joint_id, position in joint_positions.items():
            if 0 <= joint_id < 35:
                kp, kd = self._get_kp_kd(joint_id)
                cmd.motor_cmd[joint_id].mode = 1
                cmd.motor_cmd[joint_id].q = float(position)
                cmd.motor_cmd[joint_id].dq = 0.0
                cmd.motor_cmd[joint_id].tau = 0.0
                cmd.motor_cmd[joint_id].kp = kp
                cmd.motor_cmd[joint_id].kd = kd

        self.arm_sdk_pub.publish(cmd)
        return {"success": True}

    def set_left_arm_position(self, positions: list, duration: float = 1.0) -> dict:
        """왼팔 위치 제어"""
        if len(positions) != 7:
            return {"error": f"Expected 7 positions, got {len(positions)}"}

        joint_positions = {}
        for i, pos in enumerate(positions):
            joint_positions[self.JointIndex.LEFT_SHOULDER_PITCH + i] = pos

        result = self._send_arm_cmd(joint_positions, duration)
        if result.get("success"):
            return {"success": True, "action": "set_left_arm_position", "positions": positions}
        return result

    def set_right_arm_position(self, positions: list, duration: float = 1.0) -> dict:
        """오른팔 위치 제어"""
        if len(positions) != 7:
            return {"error": f"Expected 7 positions, got {len(positions)}"}

        joint_positions = {}
        for i, pos in enumerate(positions):
            joint_positions[self.JointIndex.RIGHT_SHOULDER_PITCH + i] = pos

        result = self._send_arm_cmd(joint_positions, duration)
        if result.get("success"):
            return {"success": True, "action": "set_right_arm_position", "positions": positions}
        return result

    def set_both_arms_position(self, left_positions: list, right_positions: list, duration: float = 1.0) -> dict:
        """양팔 동시 위치 제어"""
        if len(left_positions) != 7 or len(right_positions) != 7:
            return {"error": "Expected 7 positions for each arm"}

        joint_positions = {}
        for i, pos in enumerate(left_positions):
            joint_positions[self.JointIndex.LEFT_SHOULDER_PITCH + i] = pos
        for i, pos in enumerate(right_positions):
            joint_positions[self.JointIndex.RIGHT_SHOULDER_PITCH + i] = pos

        result = self._send_arm_cmd(joint_positions, duration)
        if result.get("success"):
            return {"success": True, "action": "set_both_arms_position"}
        return result

    def set_waist_position(self, yaw: float = 0.0, roll: float = 0.0, pitch: float = 0.0, duration: float = 1.0) -> dict:
        """허리 위치 제어"""
        joint_positions = {
            self.JointIndex.WAIST_YAW: yaw,
            self.JointIndex.WAIST_ROLL: roll,
            self.JointIndex.WAIST_PITCH: pitch
        }

        result = self._send_arm_cmd(joint_positions, duration)
        if result.get("success"):
            return {"success": True, "action": "set_waist_position", "yaw": yaw, "roll": roll, "pitch": pitch}
        return result

    # ============================================================
    # 5. 저수준 관절 제어 (via /lowcmd)
    # ============================================================

    def send_joint_command(self, joint_id: int, q: float = 0.0, dq: float = 0.0,
                           tau: float = 0.0, kp: float = 50.0, kd: float = 1.0) -> dict:
        """단일 관절 명령 전송"""
        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        if joint_id < 0 or joint_id >= self.NUM_MOTORS:
            return {"error": f"Invalid joint_id: {joint_id}. Must be 0-{self.NUM_MOTORS-1}"}

        cmd = self._create_low_cmd()
        cmd.motor_cmd[joint_id].mode = 1
        cmd.motor_cmd[joint_id].q = float(q)
        cmd.motor_cmd[joint_id].dq = float(dq)
        cmd.motor_cmd[joint_id].tau = float(tau)
        cmd.motor_cmd[joint_id].kp = float(kp)
        cmd.motor_cmd[joint_id].kd = float(kd)

        self.lowcmd_pub.publish(cmd)

        return {
            "success": True,
            "action": "send_joint_command",
            "joint_id": joint_id,
            "joint_name": self.JOINT_NAMES[joint_id] if joint_id < len(self.JOINT_NAMES) else "unknown",
            "q": q, "dq": dq, "tau": tau, "kp": kp, "kd": kd
        }

    def send_joint_positions(self, positions: list, kp: float = 50.0, kd: float = 1.0, duration: float = 1.0) -> dict:
        """전체 관절 위치 제어"""
        if len(positions) != self.NUM_MOTORS:
            return {"error": f"Expected {self.NUM_MOTORS} positions, got {len(positions)}"}

        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        cmd = self._create_low_cmd()

        for i in range(self.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 1
            cmd.motor_cmd[i].q = float(positions[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = float(kp)
            cmd.motor_cmd[i].kd = float(kd)

        self.lowcmd_pub.publish(cmd)

        return {
            "success": True,
            "action": "set_joint_positions",
            "num_joints": self.NUM_MOTORS
        }

    def send_leg_command(self, leg: str, positions: list, kp: float = 100.0, kd: float = 2.0) -> dict:
        """다리 관절 명령 전송"""
        if len(positions) != 6:
            return {"error": f"Expected 6 positions, got {len(positions)}"}

        if leg not in ["left", "right"]:
            return {"error": f"Invalid leg: {leg}. Must be 'left' or 'right'"}

        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        cmd = self._create_low_cmd()

        start_idx = 0 if leg == "left" else 6
        for i, pos in enumerate(positions):
            joint_id = start_idx + i
            cmd.motor_cmd[joint_id].mode = 1
            cmd.motor_cmd[joint_id].q = float(pos)
            cmd.motor_cmd[joint_id].dq = 0.0
            cmd.motor_cmd[joint_id].tau = 0.0
            cmd.motor_cmd[joint_id].kp = float(kp)
            cmd.motor_cmd[joint_id].kd = float(kd)

        self.lowcmd_pub.publish(cmd)

        return {
            "success": True,
            "action": "send_leg_command",
            "leg": leg,
            "positions": positions
        }

    # ============================================================
    # 6. 유틸리티
    # ============================================================

    def get_joint_info(self) -> dict:
        """관절 정보 조회"""
        joints = {}
        for i, name in enumerate(self.JOINT_NAMES):
            kp, kd = self._get_kp_kd(i)
            joints[name] = {
                "id": i,
                "default_kp": kp,
                "default_kd": kd
            }

        return {
            "success": True,
            "num_joints": self.NUM_MOTORS,
            "joints": joints,
            "groups": {
                "left_leg": list(range(0, 6)),
                "right_leg": list(range(6, 12)),
                "waist": list(range(12, 15)),
                "left_arm": list(range(15, 22)),
                "right_arm": list(range(22, 29))
            }
        }

    def get_pose(self) -> dict:
        """현재 위치 조회"""
        return self.get_robot_position()

    def shutdown(self):
        """종료"""
        if self.is_standing:
            self.damp()  # 안전하게 댐핑 모드로 전환
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.node.destroy_node()
