from abc import ABC, abstractmethod
from typing import Optional
import os
import time
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


class BaseRobotAdapter(ABC):
    """로봇 어댑터의 기본 추상 클래스"""

    def __init__(self, node_name: str, cmd_vel_topic: str):
        self.cmd_vel_topic = cmd_vel_topic

        # ROS 노드 초기화
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node(node_name)

        # cmd_vel Publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            self.cmd_vel_topic,
            QoSProfile(depth=10)
        )

        # 로봇 파라미터 (서브클래스에서 오버라이드)
        self.max_linear_speed = 1.0    # m/s
        self.max_angular_speed = 1.0   # rad/s
        self.rate_hz = 10              # 제어 주기

    def _spin_once(self):
        """ROS 콜백 한 번 처리"""
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def _clamp_linear_speed(self, speed: float) -> float:
        """선속도 제한"""
        return min(abs(speed), self.max_linear_speed)

    def _clamp_angular_speed(self, speed: float) -> float:
        """각속도 제한"""
        return min(abs(speed), self.max_angular_speed)

    def _publish_for_duration(self, twist: Twist, duration: float):
        """지정 시간 동안 twist 메시지 발행"""
        num_publishes = int(duration * self.rate_hz)
        for _ in range(num_publishes):
            self.cmd_vel_pub.publish(twist)
            time.sleep(1.0 / self.rate_hz)

    # ============== 필수 구현 메서드 ==============

    @abstractmethod
    def get_pose(self) -> dict:
        """현재 로봇 위치/방향 조회"""
        pass

    # ============== 공통 이동 메서드 ==============

    def move_forward(self, speed: float = 1.0, duration: float = 1.0) -> dict:
        """직진 이동"""
        speed = self._clamp_linear_speed(speed)

        twist = Twist()
        twist.linear.x = float(speed)

        self._publish_for_duration(twist, duration)
        self.stop()

        return {
            "success": True,
            "action": "move_forward",
            "speed": speed,
            "duration": duration
        }

    def move_backward(self, speed: float = 1.0, duration: float = 1.0) -> dict:
        """후진 이동"""
        speed = self._clamp_linear_speed(speed)

        twist = Twist()
        twist.linear.x = -float(speed)

        self._publish_for_duration(twist, duration)
        self.stop()

        return {
            "success": True,
            "action": "move_backward",
            "speed": speed,
            "duration": duration
        }

    def rotate(self, angle: float = 1.57, angular_speed: float = 1.0, direction: str = "left") -> dict:
        """제자리 회전"""
        angular_speed = self._clamp_angular_speed(angular_speed)
        duration = abs(angle) / angular_speed

        twist = Twist()
        if direction.lower() == "right":
            twist.angular.z = -float(angular_speed)
        else:
            twist.angular.z = float(angular_speed)

        self._publish_for_duration(twist, duration)
        self.stop()

        return {
            "success": True,
            "action": "rotate",
            "angle": angle,
            "direction": direction,
            "duration": duration
        }

    def stop(self) -> dict:
        """정지"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        return {"success": True, "action": "stop"}

    def move_with_velocity(self, linear_x: float = 0.0, linear_y: float = 0.0,
                           angular_z: float = 0.0, duration: float = 1.0) -> dict:
        """속도 직접 지정 이동"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)

        self._publish_for_duration(twist, duration)
        self.stop()

        return {
            "success": True,
            "action": "move_with_velocity",
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": angular_z,
            "duration": duration
        }

    def shutdown(self):
        """어댑터 종료"""
        self.stop()
        self.node.destroy_node()

    # ============== ROS 2 시스템 조회 메서드 ==============

    def get_topics(self) -> dict:
        """사용 가능한 모든 ROS 2 토픽 목록 조회"""
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
            topics = [{"name": name, "types": types} for name, types in topic_names_and_types]
            return {
                "success": True,
                "topics": topics,
                "count": len(topics)
            }
        except Exception as e:
            return {"error": str(e)}

    def get_topic_type(self, topic: str) -> dict:
        """특정 토픽의 메시지 타입 조회"""
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
            for name, types in topic_names_and_types:
                if name == topic:
                    return {
                        "success": True,
                        "topic": name,
                        "types": types
                    }
            return {"error": f"Topic '{topic}' not found"}
        except Exception as e:
            return {"error": str(e)}

    def get_topic_publishers_count(self, topic: str) -> dict:
        """특정 토픽의 publisher 개수 조회"""
        try:
            count = self.node.count_publishers(topic)
            return {
                "success": True,
                "topic": topic,
                "publishers_count": count
            }
        except Exception as e:
            return {"error": str(e)}

    def get_topic_subscribers_count(self, topic: str) -> dict:
        """특정 토픽의 subscriber 개수 조회"""
        try:
            count = self.node.count_subscribers(topic)
            return {
                "success": True,
                "topic": topic,
                "subscribers_count": count
            }
        except Exception as e:
            return {"error": str(e)}

    def get_services(self) -> dict:
        """사용 가능한 모든 ROS 2 서비스 목록 조회"""
        try:
            service_names_and_types = self.node.get_service_names_and_types()
            services = [{"name": name, "types": types} for name, types in service_names_and_types]
            return {
                "success": True,
                "services": services,
                "count": len(services)
            }
        except Exception as e:
            return {"error": str(e)}

    def get_service_type(self, service: str) -> dict:
        """특정 서비스의 타입 조회"""
        try:
            service_names_and_types = self.node.get_service_names_and_types()
            for name, types in service_names_and_types:
                if name == service:
                    return {
                        "success": True,
                        "service": name,
                        "types": types
                    }
            return {"error": f"Service '{service}' not found"}
        except Exception as e:
            return {"error": str(e)}

    def get_nodes(self) -> dict:
        """현재 실행 중인 모든 ROS 2 노드 목록 조회"""
        try:
            node_names = self.node.get_node_names()
            return {
                "success": True,
                "nodes": node_names,
                "count": len(node_names)
            }
        except Exception as e:
            return {"error": str(e)}

    def get_node_parameters(self, node_name: str) -> dict:
        """특정 노드의 파라미터 목록 조회"""
        try:
            return {
                "error": "Parameter operations require additional implementation",
                "note": "Use 'ros2 param list' CLI for now"
            }
        except Exception as e:
            return {"error": str(e)}

    def get_ros_info(self) -> dict:
        """ROS 2 시스템 정보 조회"""
        try:
            return {
                "success": True,
                "ros_version": "ROS 2",
                "rclpy_version": rclpy.__version__ if hasattr(rclpy, '__version__') else "unknown",
                "ros_distro": os.getenv("ROS_DISTRO", "unknown"),
                "node_name": self.node.get_name()
            }
        except Exception as e:
            return {"error": str(e)}

    def check_ros_connection(self) -> dict:
        """ROS 2 연결 상태 확인"""
        try:
            return {
                "success": True,
                "ros_initialized": rclpy.ok(),
                "node_name": self.node.get_name()
            }
        except Exception as e:
            return {"error": str(e)}
