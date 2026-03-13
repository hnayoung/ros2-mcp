from __future__ import annotations

"""
G1 Humanoid Robot Controller

저수준 G1 제어 전용 구현
- Low-Level: /lowcmd, /lowstate
"""

import struct
import sys
import threading
import time
from typing import Optional, List

# ROS 2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Unitree ROS2 메시지 타입
try:
    from unitree_hg.msg import HandCmd, HandState, LowCmd, LowState, MotorCmd
    UNITREE_ROS2_AVAILABLE = True
except ImportError:
    UNITREE_ROS2_AVAILABLE = False
    print("[WARN] unitree messages not found.", file=sys.stderr)
    print(
        "       Source: source /home/na0/workspace/ros2-mcp/ros2/unitree_ros2/setup_local.sh",
        file=sys.stderr,
    )


class HGLowCmdCRC:
    """unitree_hg.msg.LowCmd CRC 계산기"""

    _PACK_FMT = "<2B2x" + "B3x5fI" * 35 + "5I"

    def crc(self, cmd) -> int:
        values = [cmd.mode_pr, cmd.mode_machine]

        for i in range(35):
            motor = cmd.motor_cmd[i]
            values.append(motor.mode)
            values.append(motor.q)
            values.append(motor.dq)
            values.append(motor.tau)
            values.append(motor.kp)
            values.append(motor.kd)
            values.append(motor.reserve)

        values.extend(cmd.reserve)
        values.append(0)

        packed = struct.pack(self._PACK_FMT, *values)
        data = []
        calc_len = (len(packed) >> 2) - 1
        for i in range(calc_len):
            base = i * 4
            word = (
                (packed[base + 3] << 24)
                | (packed[base + 2] << 16)
                | (packed[base + 1] << 8)
                | packed[base]
            )
            data.append(word)

        return self._crc32_core(data)

    @staticmethod
    def _crc32_core(data: List[int]) -> int:
        crc = 0xFFFFFFFF
        polynomial = 0x04C11DB7

        for current in data:
            bit = 1 << 31
            for _ in range(32):
                if crc & 0x80000000:
                    crc = ((crc << 1) & 0xFFFFFFFF) ^ polynomial
                else:
                    crc = (crc << 1) & 0xFFFFFFFF

                if current & bit:
                    crc ^= polynomial

                bit >>= 1

        return crc


class G1Robot:
    """
    Unitree G1 휴머노이드 로봇 저수준 제어

    토픽:
    - /lowcmd: 저수준 모터 명령 (Publish)
    - /lowstate: 저수준 모터 상태 (Subscribe)
    """

    NUM_MOTORS = 29
    NUM_HAND_MOTORS = 6
    DEFAULT_BODY_POSE = [0.0] * NUM_MOTORS
    LEFT_HAND_CMD_TOPIC = "/inspire/left/cmd"
    RIGHT_HAND_CMD_TOPIC = "/inspire/right/cmd"
    LEFT_HAND_STATE_TOPIC = "/lf/inspire/left/state"
    RIGHT_HAND_STATE_TOPIC = "/lf/inspire/right/state"

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

    # 관절 이름(29)
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
    LEFT_HAND_JOINT_NAMES = [
        "L_thumb_proximal_yaw_joint",
        "L_thumb_proximal_pitch_joint",
        "L_index_proximal_joint",
        "L_middle_proximal_joint",
        "L_ring_proximal_joint",
        "L_pinky_proximal_joint",
    ]
    RIGHT_HAND_JOINT_NAMES = [
        "R_thumb_proximal_yaw_joint",
        "R_thumb_proximal_pitch_joint",
        "R_index_proximal_joint",
        "R_middle_proximal_joint",
        "R_ring_proximal_joint",
        "R_pinky_proximal_joint",
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
        self.crc = HGLowCmdCRC()
        self._mode_machine_logged = False
        self._shutdown = False
        self.executor: Optional[SingleThreadedExecutor] = None
        self.executor_thread: Optional[threading.Thread] = None

        self.node = self._create_ros_node()

        # 상태 변수
        self.low_state: Optional[LowState] = None
        self.left_hand_state: Optional[HandState] = None
        self.right_hand_state: Optional[HandState] = None

        # ROS2 초기화
        if UNITREE_ROS2_AVAILABLE:
            self._init_ros2_interfaces()
            self._start_executor()
        else:
            print("[G1Robot] WARNING: unitree messages not available!", file=sys.stderr)

    def _create_ros_node(self):
        """stale RMW 상태를 정리하며 ROS2 node 생성"""
        last_error = None
        for attempt in range(2):
            try:
                if not rclpy.ok():
                    rclpy.init()
                return rclpy.create_node("g1_mcp_robot")
            except Exception as exc:
                last_error = exc
                print(
                    f"[G1Robot] ROS node creation failed (attempt {attempt + 1}/2): {exc}",
                    file=sys.stderr,
                )
                try:
                    rclpy.try_shutdown()
                except Exception:
                    pass
                time.sleep(0.1)

        raise RuntimeError(f"failed to initialize ROS2 node: {last_error}") from last_error

    def _init_ros2_interfaces(self):
        """ROS2 인터페이스 초기화"""

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
        self.left_hand_cmd_pub = self.node.create_publisher(
            HandCmd, self.LEFT_HAND_CMD_TOPIC, qos_best_effort
        )
        self.right_hand_cmd_pub = self.node.create_publisher(
            HandCmd, self.RIGHT_HAND_CMD_TOPIC, qos_best_effort
        )

        # ===== Subscribers =====
        # 저수준 모터 상태
        self.lowstate_sub = self.node.create_subscription(
            LowState, "/lowstate", self._lowstate_callback, qos_best_effort
        )
        self.left_hand_state_sub = self.node.create_subscription(
            HandState, self.LEFT_HAND_STATE_TOPIC, self._left_hand_state_callback, qos_best_effort
        )
        self.right_hand_state_sub = self.node.create_subscription(
            HandState, self.RIGHT_HAND_STATE_TOPIC, self._right_hand_state_callback, qos_best_effort
        )

        print("[G1Robot] ROS2 interfaces initialized", file=sys.stderr)
        print(
            f"  Publishers: /lowcmd, {self.LEFT_HAND_CMD_TOPIC}, {self.RIGHT_HAND_CMD_TOPIC}",
            file=sys.stderr,
        )
        print(
            f"  Subscribers: /lowstate, {self.LEFT_HAND_STATE_TOPIC}, {self.RIGHT_HAND_STATE_TOPIC}",
            file=sys.stderr,
        )

    def _start_executor(self):
        """백그라운드에서 ROS callback을 지속적으로 처리"""
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            name="g1_robot_executor",
            daemon=True,
        )
        self.executor_thread.start()
        print("[G1Robot] ROS executor thread started", file=sys.stderr)

    # ============================================================
    # 콜백 함수
    # ============================================================

    def _lowstate_callback(self, msg: LowState):
        """저수준 상태 콜백"""
        self.low_state = msg
        if not self._mode_machine_logged and hasattr(msg, "mode_machine"):
            print(f"[G1Robot] low_state.mode_machine = {int(msg.mode_machine)}", file=sys.stderr)
            self._mode_machine_logged = True

    def _left_hand_state_callback(self, msg: HandState):
        self.left_hand_state = msg

    def _right_hand_state_callback(self, msg: HandState):
        self.right_hand_state = msg

    # ============================================================
    # 헬퍼 함수
    # ============================================================

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

    def _publish_lowcmd(self, cmd: LowCmd) -> None:
        """CRC를 채운 뒤 /lowcmd로 발행"""
        cmd.crc = self.crc.crc(cmd)
        self.lowcmd_pub.publish(cmd)

    @staticmethod
    def _publish_handcmd(publisher, cmd: HandCmd) -> None:
        publisher.publish(cmd)

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
        if self.executor_thread and self.executor_thread.is_alive():
            return
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

    def get_hand_states(self) -> dict:
        """손 상태 조회"""
        self._spin_once()

        def _extract(state: Optional[HandState], joint_names: List[str]) -> dict:
            if state is None:
                return {"received": False}
            positions = [round(float(m.q), 4) for m in state.motor_state[: len(joint_names)]]
            velocities = [round(float(m.dq), 4) for m in state.motor_state[: len(joint_names)]]
            efforts = [round(float(m.tau_est), 4) for m in state.motor_state[: len(joint_names)]]
            return {
                "received": True,
                "joint_names": joint_names,
                "positions": positions,
                "velocities": velocities,
                "efforts": efforts,
            }

        return {
            "success": True,
            "left_hand": _extract(self.left_hand_state, self.LEFT_HAND_JOINT_NAMES),
            "right_hand": _extract(self.right_hand_state, self.RIGHT_HAND_JOINT_NAMES),
        }

    # ============================================================
    # 2. 저수준 관절 제어 (via /lowcmd)
    # ============================================================

    def send_joint_command(self, joint_id: int, q: float = 0.0, dq: float = 0.0,
                           tau: float = 0.0, kp: float = 50.0, kd: float = 1.0,
                           duration: float = 0.3) -> dict:
        """단일 관절 목표를 반영하되, 나머지 관절은 기본 자세를 유지하며 전신 포즈로 전송"""
        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        if joint_id < 0 or joint_id >= self.NUM_MOTORS:
            return {"error": f"Invalid joint_id: {joint_id}. Must be 0-{self.NUM_MOTORS-1}"}

        self._spin_once()
        start_positions = self.DEFAULT_BODY_POSE.copy()
        if self.low_state is not None:
            start_positions = [float(self.low_state.motor_state[i].q) for i in range(self.NUM_MOTORS)]

        target_positions = self.DEFAULT_BODY_POSE.copy()
        target_positions[joint_id] = float(q)

        if duration < 0:
            return {"error": "duration must be >= 0"}

        if duration == 0:
            position_result = self.send_joint_positions(target_positions, kp=kp, kd=kd)
            if not position_result.get("success"):
                return position_result
            steps = 1
        else:
            step_dt = 0.02
            steps = max(1, int(duration / step_dt))
            for step_idx in range(1, steps + 1):
                alpha = step_idx / steps
                blended_positions = [
                    start_positions[i] + (target_positions[i] - start_positions[i]) * alpha
                    for i in range(self.NUM_MOTORS)
                ]
                position_result = self.send_joint_positions(blended_positions, kp=kp, kd=kd)
                if not position_result.get("success"):
                    return position_result
                if step_idx < steps:
                    time.sleep(duration / steps)

        return {
            "success": True,
            "action": "send_joint_command",
            "mode": "default_pose_hold_with_blend",
            "joint_id": joint_id,
            "joint_name": self.JOINT_NAMES[joint_id] if joint_id < len(self.JOINT_NAMES) else "unknown",
            "q": q,
            "dq": dq,
            "tau": tau,
            "kp": kp,
            "kd": kd,
            "duration": duration,
            "blend_steps": steps,
            "hold_others": "default_pose",
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

        self._publish_lowcmd(cmd)

        return {
            "success": True,
            "action": "set_joint_positions",
            "num_joints": self.NUM_MOTORS
        }

    def send_hand_command(self, hand: str, positions: list, kp: float = 1.5, kd: float = 0.1) -> dict:
        """단일 손 명령 전송"""
        if len(positions) != self.NUM_HAND_MOTORS:
            return {
                "error": f"Expected {self.NUM_HAND_MOTORS} positions, got {len(positions)}"
            }

        if hand not in ["left", "right"]:
            return {"error": "hand must be 'left' or 'right'"}

        if not UNITREE_ROS2_AVAILABLE:
            return {"error": "unitree_ros2 not available"}

        cmd = HandCmd()
        cmd.motor_cmd = []
        for index, position in enumerate(positions):
            motor_cmd = MotorCmd()
            motor_cmd.mode = index
            motor_cmd.q = float(position)
            motor_cmd.dq = 0.0
            motor_cmd.tau = 0.0
            motor_cmd.kp = float(kp)
            motor_cmd.kd = float(kd)
            motor_cmd.reserve = 0
            cmd.motor_cmd.append(motor_cmd)

        publisher = self.left_hand_cmd_pub if hand == "left" else self.right_hand_cmd_pub
        self._publish_handcmd(publisher, cmd)

        return {
            "success": True,
            "action": "send_hand_command",
            "hand": hand,
            "positions": positions,
            "joint_names": self.LEFT_HAND_JOINT_NAMES if hand == "left" else self.RIGHT_HAND_JOINT_NAMES,
        }

    # ============================================================
    # 3. 유틸리티
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
            "hand_joints": {
                "left_hand": {
                    name: {"id": i}
                    for i, name in enumerate(self.LEFT_HAND_JOINT_NAMES)
                },
                "right_hand": {
                    name: {"id": i}
                    for i, name in enumerate(self.RIGHT_HAND_JOINT_NAMES)
                },
            },
            "groups": {
                "left_leg": list(range(0, 6)),
                "right_leg": list(range(6, 12)),
                "waist": list(range(12, 15)),
                "left_arm": list(range(15, 22)),
                "right_arm": list(range(22, 29)),
                "left_hand": list(range(0, self.NUM_HAND_MOTORS)),
                "right_hand": list(range(0, self.NUM_HAND_MOTORS)),
            }
        }

    def shutdown(self):
        """종료"""
        if self._shutdown:
            return
        self._shutdown = True

        if self.executor:
            self.executor.shutdown()
            if self.executor_thread and self.executor_thread.is_alive():
                self.executor_thread.join(timeout=1.0)
            self.executor = None
            self.executor_thread = None

        if getattr(self, "node", None) is None:
            return

        try:
            self.node.destroy_node()
        finally:
            self.node = None
            if rclpy.ok():
                rclpy.shutdown()
