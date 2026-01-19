"""
Unitree G1 휴머노이드 로봇 어댑터 (ROS 1 버전)

unitree_sdk2py (DDS)와 ROS 1 (rospy)을 함께 사용합니다.
"""

import math
import time
import numpy as np

# ROS 1
try:
    import rospy
    from geometry_msgs.msg import Twist
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    print("[WARN] rospy not found. ROS 1 will not work.")

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

from .base_adapter_ros1 import BaseRobotAdapterROS1, ROS1_AVAILABLE


class G1AdapterROS1(BaseRobotAdapterROS1):
    """Unitree G1 휴머노이드 로봇 제어를 위한 ROS 1 어댑터"""

    ROBOT_NAME = 'g1'
    NUM_MOTORS = 29

    class GaitMode:
        STAND = 0
        WALK = 1
        RUN = 2
        CLIMB = 3

    # 기본 서있는 자세 (G1 29DOF)
    STAND_POSE = np.array([
        0.0, 0.0, 0.0, 0.4, -0.8, 0.0,
        0.0, 0.0, 0.0, 0.4, -0.8, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
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
            node_name='g1_mcp_adapter_ros1',
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
            print("[G1AdapterROS1] DDS initialized successfully")

        except Exception as e:
            print(f"[G1AdapterROS1] DDS initialization failed: {e}")
            self.dds_initialized = False

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

    # ============== BaseRobotAdapterROS1 오버라이드 ==============

    def get_pose(self) -> dict:
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
        if mode not in [0, 1, 2, 3]:
            return {"error": f"Invalid mode: {mode}"}
        self.current_gait_mode = mode
        names = {0: "STAND", 1: "WALK", 2: "RUN", 3: "CLIMB"}
        return {"success": True, "mode": mode, "mode_name": names[mode]}

    def walk(self, direction: str = "forward", speed: float = 0.5, duration: float = 2.0) -> dict:
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
        if self.low_state and hasattr(self.low_state, 'imu_state'):
            imu = self.low_state.imu_state
            return {
                "success": True,
                "quaternion": list(imu.quaternion) if hasattr(imu, 'quaternion') else [],
                "gyroscope": list(imu.gyroscope) if hasattr(imu, 'gyroscope') else []
            }
        return {"error": "IMU not available"}

    def get_robot_status(self) -> dict:
        names = {0: "STAND", 1: "WALK", 2: "RUN", 3: "CLIMB"}
        return {
            "success": True,
            "robot_name": self.ROBOT_NAME,
            "ros_version": "ROS 1",
            "dds_connected": self.dds_initialized,
            "is_standing": self.is_standing,
            "gait_mode": names.get(self.current_gait_mode, "UNKNOWN"),
            "position": {
                "x": round(self.position_x, 2),
                "y": round(self.position_y, 2)
            }
        }

    def wave_hand(self, hand: str = "right") -> dict:
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        if not self.dds_initialized:
            return {"success": True, "action": "wave_hand", "note": "Simulated (no DDS)"}

        wave_pose = self.STAND_POSE.copy()
        if hand == "right":
            wave_pose[23] = -1.2
            wave_pose[24] = -0.5
            wave_pose[26] = 1.0
        else:
            wave_pose[15] = -1.2
            wave_pose[16] = 0.5
            wave_pose[18] = 1.0

        self._send_motor_cmd(wave_pose, kp=40.0, kd=3.0, duration=1.0)

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
        if not self.is_standing:
            self.stand_up()
            time.sleep(0.5)

        if not self.dds_initialized:
            return {"success": True, "action": "bow", "note": "Simulated"}

        bow_pose = self.STAND_POSE.copy()
        bow_pose[14] = 0.4

        self._send_motor_cmd(bow_pose, kp=40.0, kd=3.0, duration=1.0)
        time.sleep(0.5)
        self._send_motor_cmd(self.STAND_POSE, kp=50.0, kd=3.5, duration=1.0)

        return {"success": True, "action": "bow", "message": "인사 완료"}

    def stop(self) -> dict:
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        return {"success": True, "action": "stop"}

    def shutdown(self):
        if self.is_standing and self.dds_initialized:
            self.sit_down()
        super().shutdown()
