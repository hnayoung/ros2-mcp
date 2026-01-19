#!/usr/bin/env python3
"""
G1 ROS 2 토픽/서비스 테스트 스크립트
MCP 서버 없이 독립적으로 실행하여 ROS 2 인터페이스 확인
"""

import sys
import time
import json

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Unitree SDK
try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelSubscriber
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False
    print("[WARN] unitree_sdk2py not available")


class G1ROS2Bridge(Node):
    """G1 DDS -> ROS 2 브릿지"""

    NUM_MOTORS = 29

    def __init__(self):
        super().__init__('g1_ros2_bridge')

        self.low_state = None

        # ROS 2 Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/g1/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/g1/imu', 10)
        self.status_pub = self.create_publisher(String, '/g1/status', 10)

        # ROS 2 Services
        self.create_service(Trigger, '/g1/test', self.test_service)

        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_states)

        # DDS 초기화
        if UNITREE_SDK_AVAILABLE:
            self._init_dds()

        self.get_logger().info("G1 ROS 2 Bridge started!")
        self.get_logger().info("Topics: /g1/joint_states, /g1/imu, /g1/status")
        self.get_logger().info("Services: /g1/test")

    def _init_dds(self):
        """DDS 초기화"""
        try:
            ChannelFactoryInitialize(1, "lo")
            self.low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self.low_state_sub.Init(self._low_state_callback, 10)
            self.get_logger().info("[DDS] Connected to rt/lowstate")
        except Exception as e:
            self.get_logger().error(f"[DDS] Failed: {e}")

    def _low_state_callback(self, msg):
        self.low_state = msg

    def publish_states(self):
        """ROS 2 토픽으로 상태 퍼블리시"""

        # JointState
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
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
            joint_msg.position = [0.0] * self.NUM_MOTORS
            joint_msg.velocity = [0.0] * self.NUM_MOTORS
            joint_msg.effort = [0.0] * self.NUM_MOTORS

        self.joint_state_pub.publish(joint_msg)

        # Status
        status = {
            "dds_connected": self.low_state is not None,
            "timestamp": time.time()
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

    def test_service(self, request, response):
        response.success = True
        response.message = "G1 ROS 2 bridge is working!"
        return response


def main():
    print("=" * 50)
    print("G1 ROS 2 Bridge 테스트")
    print("=" * 50)
    print("\n시뮬레이터가 실행 중이어야 합니다.")
    print("다른 터미널에서 확인:")
    print("  ros2 topic echo /g1/joint_states")
    print("  ros2 service call /g1/test std_srvs/srv/Trigger")
    print("\nCtrl+C로 종료\n")

    rclpy.init()
    node = G1ROS2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
