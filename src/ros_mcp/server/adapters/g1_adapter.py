import math
import time
import numpy as np
from threading import Thread

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState, Imu
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Float32MultiArray

# Unitree SDK2 Python (DDS 통신)
try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False
    print("[WARN] unitree_sdk2py not found. G1 simulation will not work.")

from .base_adapter import BaseRobotAdapter


class G1Adapter(BaseRobotAdapter):
    """Unitree G1 휴머노이드 로봇 제어를 위한 어댑터 (DDS 통신)"""

    ROBOT_NAME = 'g1'
    NUM_MOTORS = 29

    # 보행 모드
    class GaitMode:
        STAND = 0
        WALK = 1
        RUN = 2
        CLIMB = 3

    # 기본 서있는 자세 (G1 29DOF)
    STAND_POSE = np.array([
        # 왼쪽 다리 (0-5)
        0.0, 0.0, 0.0, 0.4, -0.8, 0.0,
        # 오른쪽 다리 (6-11)
        0.0, 0.0, 0.0, 0.4, -0.8, 0.0,
        # 허리 (12-14)
        0.0, 0.0, 0.0,
        # 왼팔 (15-22)
        0.0, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
        # 오른팔 (23-28)
        0.0, -0.3, 0.0, 0.5, 0.0, 0.0
    ], dtype=float)

    # 앉은 자세
    SIT_POSE = np.array([
        0.0, 0.0, 0.0, 1.2, -0.8, 0.0,
        0.0, 0.0, 0.0, 1.2, -0.8, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
        0.0, -0.3, 0.0, 0.5, 0.0, 0.0
    ], dtype=float)

    def __init__(self, robot_namespace: str = ""):
        self.namespace = robot_namespace
        prefix = f"/{robot_namespace}" if robot_namespace else ""

        super().__init__(
            node_name='g1_mcp_adapter',
            cmd_vel_topic=f"{prefix}/cmd_vel"
        )

        # 파라미터
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1.0
        self.rate_hz = 50
        self.dt = 0.002

        # 상태 변수
        self.is_standing = False
        self.current_gait_mode = self.GaitMode.STAND
        self.current_pose = self.SIT_POSE.copy()

        # DDS 상태
        self.low_state = None
        self.sport_state = None
        self.dds_initialized = False

        # 위치 추적
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.yaw = 0.0

        # DDS 초기화
        if UNITREE_SDK_AVAILABLE:
            self._init_dds()

        # ROS 2 인터페이스 초기화
        self._init_ros2_interfaces()

    def _init_dds(self):
        """DDS 채널 초기화"""
        try:
            ChannelFactoryInitialize(1, "lo")

            self.low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self.low_cmd_pub.Init()

            self.low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self.low_state_sub.Init(self._low_state_callback, 10)

            self.sport_state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            self.sport_state_sub.Init(self._sport_state_callback, 10)

            self.crc = CRC()
            self.dds_initialized = True
            print("[G1Adapter] DDS initialized successfully")

        except Exception as e:
            print(f"[G1Adapter] DDS initialization failed: {e}")
            self.dds_initialized = False

    def _init_ros2_interfaces(self):
        """ROS 2 토픽, 서비스 초기화"""
        prefix = f"/{self.namespace}" if self.namespace else ""

        # ===== Publishers (토픽) =====
        # 관절 상태 퍼블리셔
        self.joint_state_pub = self.node.create_publisher(
            JointState,
            f"{prefix}/g1/joint_states",
            10
        )

        # IMU 데이터 퍼블리셔
        self.imu_pub = self.node.create_publisher(
            Imu,
            f"{prefix}/g1/imu",
            10
        )

        # 로봇 상태 퍼블리셔
        self.robot_status_pub = self.node.create_publisher(
            String,
            f"{prefix}/g1/status",
            10
        )

        # ===== Subscribers (토픽) =====
        # 관절 명령 구독자
        self.joint_cmd_sub = self.node.create_subscription(
            Float32MultiArray,
            f"{prefix}/g1/joint_cmd",
            self._joint_cmd_callback,
            10
        )

        # ===== Services (서비스) =====
        self.stand_up_srv = self.node.create_service(
            Trigger,
            f"{prefix}/g1/stand_up",
            self._stand_up_service
        )

        self.sit_down_srv = self.node.create_service(
            Trigger,
            f"{prefix}/g1/sit_down",
            self._sit_down_service
        )

        self.damping_srv = self.node.create_service(
            Trigger,
            f"{prefix}/g1/damping_mode",
            self._damping_service
        )

        self.wave_hand_srv = self.node.create_service(
            Trigger,
            f"{prefix}/g1/wave_hand",
            self._wave_hand_service
        )

        self.bow_srv = self.node.create_service(
            Trigger,
            f"{prefix}/g1/bow",
            self._bow_service
        )

        # 주기적 퍼블리시 타이머 (10Hz)
        self.publish_timer = self.node.create_timer(0.1, self._publish_states)

        print(f"[G1Adapter] ROS 2 interfaces initialized")
        print(f"  Topics: {prefix}/g1/joint_states, {prefix}/g1/imu, {prefix}/g1/status")
        print(f"  Services: {prefix}/g1/stand_up, {prefix}/g1/sit_down, {prefix}/g1/damping_mode, {prefix}/g1/wave_hand, {prefix}/g1/bow")

    def _publish_states(self):
        """주기적으로 상태 퍼블리시"""
        # JointState 퍼블리시
        joint_msg = JointState()
        joint_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_msg.name = [
            "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
            "left_knee", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
            "right_knee", "right_ankle_pitch", "right_ankle_roll",
            "waist_yaw", "waist_roll", "waist_pitch",
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
            "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
            "left_hand",
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
            "right_elbow", "right_wrist_roll", "right_wrist_pitch"
        ]

        if self.low_state:
            positions = []
            velocities = []
            efforts = []
            for i in range(self.NUM_MOTORS):
                try:
                    positions.append(float(self.low_state.motor_state[i].q))
                    velocities.append(float(self.low_state.motor_state[i].dq))
                    efforts.append(float(self.low_state.motor_state[i].tau_est))
                except:
                    positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
            joint_msg.position = positions
            joint_msg.velocity = velocities
            joint_msg.effort = efforts
        else:
            joint_msg.position = self.current_pose.tolist()
            joint_msg.velocity = [0.0] * self.NUM_MOTORS
            joint_msg.effort = [0.0] * self.NUM_MOTORS

        self.joint_state_pub.publish(joint_msg)

        # IMU 퍼블리시
        if self.low_state and hasattr(self.low_state, 'imu_state'):
            imu_msg = Imu()
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu = self.low_state.imu_state
            if hasattr(imu, 'quaternion') and len(imu.quaternion) >= 4:
                imu_msg.orientation.w = float(imu.quaternion[0])
                imu_msg.orientation.x = float(imu.quaternion[1])
                imu_msg.orientation.y = float(imu.quaternion[2])
                imu_msg.orientation.z = float(imu.quaternion[3])

            if hasattr(imu, 'gyroscope') and len(imu.gyroscope) >= 3:
                imu_msg.angular_velocity.x = float(imu.gyroscope[0])
                imu_msg.angular_velocity.y = float(imu.gyroscope[1])
                imu_msg.angular_velocity.z = float(imu.gyroscope[2])

            if hasattr(imu, 'accelerometer') and len(imu.accelerometer) >= 3:
                imu_msg.linear_acceleration.x = float(imu.accelerometer[0])
                imu_msg.linear_acceleration.y = float(imu.accelerometer[1])
                imu_msg.linear_acceleration.z = float(imu.accelerometer[2])

            self.imu_pub.publish(imu_msg)

        # 로봇 상태 퍼블리시
        import json
        status = {
            "is_standing": self.is_standing,
            "dds_connected": self.dds_initialized,
            "gait_mode": self.current_gait_mode,
            "position": {"x": self.position_x, "y": self.position_y, "z": self.position_z}
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.robot_status_pub.publish(status_msg)

    def _joint_cmd_callback(self, msg: Float32MultiArray):
        """관절 명령 콜백 (토픽으로 관절 제어)"""
        if len(msg.data) >= self.NUM_MOTORS:
            positions = list(msg.data[:self.NUM_MOTORS])
            self.set_joint_positions(positions, kp=50.0, kd=3.5, duration=1.0)

    def _stand_up_service(self, request, response):
        """stand_up 서비스 콜백"""
        result = self.stand_up()
        response.success = result.get("success", False)
        response.message = result.get("message", str(result))
        return response

    def _sit_down_service(self, request, response):
        """sit_down 서비스 콜백"""
        result = self.sit_down()
        response.success = result.get("success", False)
        response.message = result.get("message", str(result))
        return response

    def _damping_service(self, request, response):
        """damping_mode 서비스 콜백"""
        result = self.set_damping_mode()
        response.success = result.get("success", False)
        response.message = result.get("message", str(result))
        return response

    def _wave_hand_service(self, request, response):
        """wave_hand 서비스 콜백"""
        result = self.wave_hand("right")
        response.success = result.get("success", False)
        response.message = str(result)
        return response

    def _bow_service(self, request, response):
        """bow 서비스 콜백"""
        result = self.bow()
        response.success = result.get("success", False)
        response.message = result.get("message", str(result))
        return response

    def _low_state_callback(self, msg):
        self.low_state = msg

    def _sport_state_callback(self, msg):
        self.sport_state = msg
        if hasattr(msg, 'position') and len(msg.position) >= 3:
            self.position_x = msg.position[0]
            self.position_y = msg.position[1]
            self.position_z = msg.position[2]

    def _send_motor_cmd(self, target_positions, kp=50.0, kd=3.5, duration=1.0):
        """모터 명령 전송"""
        if not self.dds_initialized:
            return False

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0

        start_pose = self.current_pose.copy()
        start_time = time.time()
        num_motors = min(self.NUM_MOTORS, len(target_positions))

        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            phase = min(elapsed / duration, 1.0)
            smooth = np.tanh(phase * 2) / np.tanh(2)

            for i in range(num_motors):
                pos = (1 - smooth) * start_pose[i] + smooth * target_positions[i]
                cmd.motor_cmd[i].mode = 0x01
                cmd.motor_cmd[i].q = float(pos)
                cmd.motor_cmd[i].kp = float(kp)
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = float(kd)
                cmd.motor_cmd[i].tau = 0.0

            cmd.crc = self.crc.Crc(cmd)
            self.low_cmd_pub.Write(cmd)
            time.sleep(self.dt)

        self.current_pose = target_positions.copy()
        return True

    # ============== BaseRobotAdapter 오버라이드 ==============

    def get_pose(self) -> dict:
        self._spin_once()
        return {
            "success": True,
            "position": {
                "x": round(self.position_x, 3),
                "y": round(self.position_y, 3),
                "z": round(self.position_z, 3)
            },
            "yaw": round(self.yaw, 3),
            "dds_connected": self.dds_initialized
        }

    def stand_up(self) -> dict:
        """G1 로봇을 일어서게 합니다 (BaseRobotAdapter 오버라이드)"""
        if not self.dds_initialized:
            return {"error": "DDS not initialized. Is the simulator running?"}

        print("[G1] Standing up...")
        success = self._send_motor_cmd(self.STAND_POSE, kp=50.0, kd=3.5, duration=2.0)
        self.is_standing = success

        return {
            "success": success,
            "action": "stand_up",
            "message": "로봇이 일어섰습니다" if success else "실패"
        }

    def sit_down(self) -> dict:
        """G1 로봇을 앉게 합니다 (BaseRobotAdapter 오버라이드)"""
        if not self.dds_initialized:
            return {"error": "DDS not initialized"}

        print("[G1] Sitting down...")
        success = self._send_motor_cmd(self.SIT_POSE, kp=50.0, kd=3.5, duration=2.0)
        self.is_standing = False

        return {
            "success": success,
            "action": "sit_down",
            "message": "로봇이 앉았습니다"
        }

    def set_damping_mode(self) -> dict:
        """댐핑 모드로 전환 (BaseRobotAdapter 오버라이드)"""
        if not self.dds_initialized:
            return {"error": "DDS not initialized"}

        cmd = unitree_hg_msg_dds__LowCmd_()
        for i in range(self.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 0x00
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 5.0

        cmd.crc = self.crc.Crc(cmd)
        self.low_cmd_pub.Write(cmd)
        self.is_standing = False

        return {"success": True, "action": "damping_mode", "message": "댐핑 모드"}

    def set_gait_mode(self, mode: int) -> dict:
        """보행 모드 설정 (BaseRobotAdapter 오버라이드)"""
        if mode not in [0, 1, 2, 3]:
            return {"error": f"Invalid mode: {mode}"}
        self.current_gait_mode = mode
        names = {0: "STAND", 1: "WALK", 2: "RUN", 3: "CLIMB"}
        return {"success": True, "mode": mode, "mode_name": names[mode]}

    def walk(self, direction: str = "forward", speed: float = 0.5, duration: float = 2.0) -> dict:
        """G1 로봇 걷기 (BaseRobotAdapter 오버라이드)"""
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        speed = min(abs(speed), self.max_linear_speed)
        twist = Twist()

        if direction == "forward":
            twist.linear.x = speed
            self.position_x += speed * duration * math.cos(self.yaw)
            self.position_y += speed * duration * math.sin(self.yaw)
        elif direction == "backward":
            twist.linear.x = -speed
            self.position_x -= speed * duration * math.cos(self.yaw)
            self.position_y -= speed * duration * math.sin(self.yaw)
        elif direction == "left":
            twist.linear.y = speed * 0.5
        elif direction == "right":
            twist.linear.y = -speed * 0.5
        else:
            return {"error": f"Invalid direction: {direction}"}

        self._publish_for_duration(twist, duration)
        self.stop()

        return {
            "success": True,
            "action": "walk",
            "direction": direction,
            "speed": speed,
            "duration": duration
        }

    def turn(self, angle: float = 90.0, direction: str = "left") -> dict:
        """G1 로봇 제자리 회전 (BaseRobotAdapter 오버라이드)"""
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        angle_rad = math.radians(abs(angle))
        duration = angle_rad / self.max_angular_speed

        twist = Twist()
        if direction == "left":
            twist.angular.z = self.max_angular_speed
            self.yaw += angle_rad
        else:
            twist.angular.z = -self.max_angular_speed
            self.yaw -= angle_rad

        self._publish_for_duration(twist, duration)
        self.stop()

        return {
            "success": True,
            "action": "turn",
            "angle": angle,
            "direction": direction
        }

    def walk_to(self, x: float, y: float) -> dict:
        """목표 위치로 걸어가기 (BaseRobotAdapter 오버라이드)"""
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        dx = x - self.position_x
        dy = y - self.position_y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)

        angle_diff = target_angle - self.yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) > 0.1:
            dir = "left" if angle_diff > 0 else "right"
            self.turn(math.degrees(abs(angle_diff)), dir)

        if distance > 0.1:
            self.walk("forward", 0.5, distance / 0.5)

        return {
            "success": True,
            "action": "walk_to",
            "target": {"x": x, "y": y},
            "distance": round(distance, 2)
        }

    def get_joint_states(self) -> dict:
        """관절 상태 조회 (BaseRobotAdapter 오버라이드)"""
        if self.low_state:
            positions = []
            for i in range(min(self.NUM_MOTORS, 35)):
                try:
                    positions.append(round(self.low_state.motor_state[i].q, 4))
                except:
                    break
            return {"success": True, "positions": positions}
        return {"success": True, "current_target": self.current_pose.tolist()}

    def get_imu_data(self) -> dict:
        """IMU 센서 데이터 조회 (BaseRobotAdapter 오버라이드)"""
        if self.low_state and hasattr(self.low_state, 'imu_state'):
            imu = self.low_state.imu_state
            return {
                "success": True,
                "quaternion": list(imu.quaternion) if hasattr(imu, 'quaternion') else [],
                "gyroscope": list(imu.gyroscope) if hasattr(imu, 'gyroscope') else []
            }
        return {"error": "IMU not available"}

    def get_robot_status(self) -> dict:
        """로봇 상태 요약 (BaseRobotAdapter 오버라이드)"""
        names = {0: "STAND", 1: "WALK", 2: "RUN", 3: "CLIMB"}
        return {
            "success": True,
            "robot_name": self.ROBOT_NAME,
            "dds_connected": self.dds_initialized,
            "is_standing": self.is_standing,
            "gait_mode": names.get(self.current_gait_mode, "UNKNOWN"),
            "position": {
                "x": round(self.position_x, 2),
                "y": round(self.position_y, 2)
            }
        }

    def wave_hand(self, hand: str = "right") -> dict:
        """손 흔들기 (BaseRobotAdapter 오버라이드)"""
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        if not self.dds_initialized:
            return {"success": True, "action": "wave_hand", "note": "Simulated (no DDS)"}

        wave_pose = self.STAND_POSE.copy()
        if hand == "right":
            wave_pose[23] = -1.2  # shoulder pitch
            wave_pose[24] = -0.5  # shoulder roll
            wave_pose[26] = 1.0   # elbow
        else:
            wave_pose[15] = -1.2
            wave_pose[16] = 0.5
            wave_pose[18] = 1.0

        self._send_motor_cmd(wave_pose, kp=40.0, kd=3.0, duration=1.0)

        # 흔들기
        for _ in range(3):
            if hand == "right":
                wave_pose[25] = 0.5
            else:
                wave_pose[17] = -0.5
            self._send_motor_cmd(wave_pose, kp=30.0, kd=2.0, duration=0.25)

            if hand == "right":
                wave_pose[25] = -0.5
            else:
                wave_pose[17] = 0.5
            self._send_motor_cmd(wave_pose, kp=30.0, kd=2.0, duration=0.25)

        self._send_motor_cmd(self.STAND_POSE, kp=50.0, kd=3.5, duration=1.0)

        return {"success": True, "action": "wave_hand", "hand": hand}

    def bow(self) -> dict:
        """인사하기 (BaseRobotAdapter 오버라이드)"""
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        if not self.dds_initialized:
            return {"success": True, "action": "bow", "note": "Simulated"}

        bow_pose = self.STAND_POSE.copy()
        bow_pose[14] = 0.4  # waist pitch

        self._send_motor_cmd(bow_pose, kp=40.0, kd=3.0, duration=1.0)
        time.sleep(0.5)
        self._send_motor_cmd(self.STAND_POSE, kp=50.0, kd=3.5, duration=1.0)

        return {"success": True, "action": "bow", "message": "인사 완료"}

    def stop(self) -> dict:
        """정지 (BaseRobotAdapter 오버라이드)"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        return {"success": True, "action": "stop"}

    # ============== 저수준 명령 (Low-level Commands) ==============

    def set_joint_position(self, joint_id: int, position: float, kp: float = 50.0, kd: float = 3.5) -> dict:
        """
        단일 관절 위치 제어

        Args:
            joint_id: 관절 인덱스 (0-28)
            position: 목표 위치 (rad)
            kp: 위치 게인
            kd: 속도 게인
        """
        if not self.dds_initialized:
            return {"error": "DDS not initialized"}
        if joint_id < 0 or joint_id >= self.NUM_MOTORS:
            return {"error": f"Invalid joint_id: {joint_id}. Must be 0-{self.NUM_MOTORS-1}"}

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0

        # 다른 관절은 현재 위치 유지
        for i in range(self.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = float(self.current_pose[i])
            cmd.motor_cmd[i].kp = float(kp)
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = float(kd)
            cmd.motor_cmd[i].tau = 0.0

        # 타겟 관절만 변경
        cmd.motor_cmd[joint_id].q = float(position)

        cmd.crc = self.crc.Crc(cmd)
        self.low_cmd_pub.Write(cmd)

        self.current_pose[joint_id] = position

        return {
            "success": True,
            "action": "set_joint_position",
            "joint_id": joint_id,
            "position": position,
            "kp": kp,
            "kd": kd
        }

    def set_joint_positions(self, positions: list, kp: float = 50.0, kd: float = 3.5, duration: float = 1.0) -> dict:
        """
        모든 관절 위치 제어

        Args:
            positions: 29개 관절의 목표 위치 리스트 (rad)
            kp: 위치 게인
            kd: 속도 게인
            duration: 이동 시간 (초)
        """
        if not self.dds_initialized:
            return {"error": "DDS not initialized"}
        if len(positions) != self.NUM_MOTORS:
            return {"error": f"positions must have {self.NUM_MOTORS} elements, got {len(positions)}"}

        target = np.array(positions, dtype=float)
        success = self._send_motor_cmd(target, kp=kp, kd=kd, duration=duration)

        return {
            "success": success,
            "action": "set_joint_positions",
            "num_joints": self.NUM_MOTORS,
            "duration": duration
        }

    def set_joint_torque(self, joint_id: int, torque: float) -> dict:
        """
        단일 관절 토크 제어 (위치 제어 없이)

        Args:
            joint_id: 관절 인덱스 (0-28)
            torque: 토크 값 (Nm)
        """
        if not self.dds_initialized:
            return {"error": "DDS not initialized"}
        if joint_id < 0 or joint_id >= self.NUM_MOTORS:
            return {"error": f"Invalid joint_id: {joint_id}"}

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0

        for i in range(self.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0

        cmd.motor_cmd[joint_id].tau = float(torque)

        cmd.crc = self.crc.Crc(cmd)
        self.low_cmd_pub.Write(cmd)

        return {
            "success": True,
            "action": "set_joint_torque",
            "joint_id": joint_id,
            "torque": torque
        }

    def send_low_cmd(self, joint_cmds: list) -> dict:
        """
        저수준 명령 직접 전송

        Args:
            joint_cmds: 관절 명령 리스트. 각 항목은 dict:
                {
                    "id": 관절 인덱스,
                    "q": 목표 위치 (rad),
                    "kp": 위치 게인,
                    "dq": 목표 속도 (rad/s),
                    "kd": 속도 게인,
                    "tau": 토크 (Nm)
                }
        """
        if not self.dds_initialized:
            return {"error": "DDS not initialized"}

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0

        # 기본값: 모든 모터 비활성화
        for i in range(self.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0

        # 사용자 명령 적용
        for jcmd in joint_cmds:
            i = jcmd.get("id", 0)
            if 0 <= i < self.NUM_MOTORS:
                cmd.motor_cmd[i].q = float(jcmd.get("q", 0.0))
                cmd.motor_cmd[i].kp = float(jcmd.get("kp", 0.0))
                cmd.motor_cmd[i].dq = float(jcmd.get("dq", 0.0))
                cmd.motor_cmd[i].kd = float(jcmd.get("kd", 0.0))
                cmd.motor_cmd[i].tau = float(jcmd.get("tau", 0.0))

        cmd.crc = self.crc.Crc(cmd)
        self.low_cmd_pub.Write(cmd)

        return {
            "success": True,
            "action": "send_low_cmd",
            "num_commands": len(joint_cmds)
        }

    def get_joint_info(self) -> dict:
        """관절 이름과 인덱스 매핑 정보 반환"""
        joint_names = [
            # 왼쪽 다리 (0-5)
            "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
            "left_knee", "left_ankle_pitch", "left_ankle_roll",
            # 오른쪽 다리 (6-11)
            "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
            "right_knee", "right_ankle_pitch", "right_ankle_roll",
            # 허리 (12-14)
            "waist_yaw", "waist_roll", "waist_pitch",
            # 왼팔 (15-22)
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
            "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
            "left_hand",
            # 오른팔 (23-28)
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
            "right_elbow", "right_wrist_roll", "right_wrist_pitch"
        ]

        joints = {}
        for i, name in enumerate(joint_names):
            joints[name] = {
                "id": i,
                "current_position": round(self.current_pose[i], 4) if i < len(self.current_pose) else 0.0
            }

        return {
            "success": True,
            "num_joints": self.NUM_MOTORS,
            "joints": joints
        }

    def shutdown(self):
        if self.is_standing and self.dds_initialized:
            self.sit_down()
        super().shutdown()
