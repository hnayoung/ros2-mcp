#!/usr/bin/env python3
"""
G1 MuJoCo 시뮬레이션 <-> ROS 2 브릿지

unitree_mujoco 시뮬레이터의 DDS 메시지를 ROS 2 토픽으로 변환합니다.
이를 통해 MCP 서버가 시뮬레이션된 G1 로봇을 제어할 수 있습니다.
"""

import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32

# Unitree SDK2 Python
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_


class G1ROS2Bridge(Node):
    """G1 시뮬레이터와 ROS 2를 연결하는 브릿지 노드"""

    def __init__(self):
        super().__init__('g1_ros2_bridge')

        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ROS 2 퍼블리셔 (시뮬레이터 -> ROS 2)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', qos)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos)

        # ROS 2 서브스크라이버 (ROS 2 -> 시뮬레이터)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos
        )
        self.gait_mode_sub = self.create_subscription(
            Float32, '/gait_mode', self.gait_mode_callback, qos
        )
        self.posture_mode_sub = self.create_subscription(
            Float32, '/posture_mode', self.posture_mode_callback, qos
        )

        # 상태 변수
        self.current_cmd_vel = Twist()
        self.current_gait_mode = 0
        self.current_posture_mode = 0
        self.robot_state = None

        # 위치 추적 (간단한 오도메트리)
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.9  # G1 standing height
        self.yaw = 0.0
        self.last_time = time.time()

        # DDS 초기화
        self._init_dds()

        # 타이머
        self.create_timer(0.02, self.publish_robot_state)  # 50Hz

        self.get_logger().info('G1 ROS2 Bridge initialized')

    def _init_dds(self):
        """DDS 채널 초기화"""
        try:
            ChannelFactoryInitialize(1, "lo")  # domain_id=1, interface=lo

            # LowState 구독 (시뮬레이터로부터)
            self.low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self.low_state_sub.Init(self._low_state_callback, 10)

            # LowCmd 퍼블리셔 (시뮬레이터로)
            self.low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self.low_cmd_pub.Init()

            self.get_logger().info('DDS channels initialized')
        except Exception as e:
            self.get_logger().error(f'DDS initialization failed: {e}')

    def _low_state_callback(self, msg: LowState_):
        """시뮬레이터로부터 로봇 상태 수신"""
        self.robot_state = msg

    def cmd_vel_callback(self, msg: Twist):
        """cmd_vel 메시지 수신"""
        self.current_cmd_vel = msg

        # 간단한 오도메트리 업데이트
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 위치 업데이트
        self.position_x += msg.linear.x * math.cos(self.yaw) * dt
        self.position_y += msg.linear.x * math.sin(self.yaw) * dt
        self.yaw += msg.angular.z * dt

    def gait_mode_callback(self, msg: Float32):
        """보행 모드 변경"""
        self.current_gait_mode = int(msg.data)
        self.get_logger().info(f'Gait mode changed to: {self.current_gait_mode}')

    def posture_mode_callback(self, msg: Float32):
        """자세 모드 변경"""
        self.current_posture_mode = int(msg.data)
        self.get_logger().info(f'Posture mode changed to: {self.current_posture_mode}')

    def publish_robot_state(self):
        """ROS 2로 로봇 상태 퍼블리시"""
        now = self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = self.position_z

        # Yaw to quaternion
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)

        odom.twist.twist = self.current_cmd_vel
        self.odom_pub.publish(odom)

        # Joint States
        if self.robot_state:
            joint_state = JointState()
            joint_state.header.stamp = now

            # G1 has 29 DOF joints
            joint_names = [
                'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
                'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
                'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
                'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
                'waist_yaw', 'waist_roll', 'waist_pitch',
                'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
                'left_elbow', 'left_wrist_yaw', 'left_wrist_roll', 'left_wrist_pitch',
                'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
                'right_elbow', 'right_wrist_yaw', 'right_wrist_roll', 'right_wrist_pitch'
            ]
            joint_state.name = joint_names[:len(self.robot_state.motor_state)]

            positions = []
            velocities = []
            efforts = []

            for i, motor in enumerate(self.robot_state.motor_state):
                if i < len(joint_names):
                    positions.append(motor.q)
                    velocities.append(motor.dq)
                    efforts.append(motor.tau_est)

            joint_state.position = positions
            joint_state.velocity = velocities
            joint_state.effort = efforts

            self.joint_states_pub.publish(joint_state)

            # IMU
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'

            imu_data = self.robot_state.imu_state
            imu.orientation.x = imu_data.quaternion[1]
            imu.orientation.y = imu_data.quaternion[2]
            imu.orientation.z = imu_data.quaternion[3]
            imu.orientation.w = imu_data.quaternion[0]

            imu.angular_velocity.x = imu_data.gyroscope[0]
            imu.angular_velocity.y = imu_data.gyroscope[1]
            imu.angular_velocity.z = imu_data.gyroscope[2]

            imu.linear_acceleration.x = imu_data.accelerometer[0]
            imu.linear_acceleration.y = imu_data.accelerometer[1]
            imu.linear_acceleration.z = imu_data.accelerometer[2]

            self.imu_pub.publish(imu)


def main():
    rclpy.init()
    node = G1ROS2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
