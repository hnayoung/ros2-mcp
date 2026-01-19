"""
ROS 1 (rospy) 기반 로봇 어댑터 베이스 클래스
"""

from abc import ABC, abstractmethod
import os
import time

try:
    import rospy
    from geometry_msgs.msg import Twist
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    print("[WARN] rospy not found. ROS 1 will not work.")


class BaseRobotAdapterROS1(ABC):
    """ROS 1 로봇 어댑터의 기본 추상 클래스"""

    def __init__(self, node_name: str, cmd_vel_topic: str):
        if not ROS1_AVAILABLE:
            raise RuntimeError("rospy is not available")

        self.cmd_vel_topic = cmd_vel_topic

        # ROS 노드 초기화
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True)

        self.node_name = node_name

        # cmd_vel Publisher
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic,
            Twist,
            queue_size=10
        )

        # 로봇 파라미터 (서브클래스에서 오버라이드)
        self.max_linear_speed = 1.0    # m/s
        self.max_angular_speed = 1.0   # rad/s
        self.rate_hz = 10              # 제어 주기

    def _clamp_linear_speed(self, speed: float) -> float:
        """선속도 제한"""
        return min(abs(speed), self.max_linear_speed)

    def _clamp_angular_speed(self, speed: float) -> float:
        """각속도 제한"""
        return min(abs(speed), self.max_angular_speed)

    def _publish_for_duration(self, twist: Twist, duration: float):
        """지정 시간 동안 twist 메시지 발행"""
        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

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

    # ============== ROS 1 시스템 조회 메서드 ==============

    def get_topics(self) -> dict:
        """사용 가능한 모든 ROS 토픽 목록 조회"""
        try:
            topics = rospy.get_published_topics()
            topic_list = [{"name": name, "types": [msg_type]} for name, msg_type in topics]
            return {
                "success": True,
                "topics": topic_list,
                "count": len(topic_list)
            }
        except Exception as e:
            return {"error": str(e)}

    def get_topic_type(self, topic: str) -> dict:
        """특정 토픽의 메시지 타입 조회"""
        try:
            topics = rospy.get_published_topics()
            for name, msg_type in topics:
                if name == topic:
                    return {
                        "success": True,
                        "topic": name,
                        "types": [msg_type]
                    }
            return {"error": f"Topic '{topic}' not found"}
        except Exception as e:
            return {"error": str(e)}

    def get_nodes(self) -> dict:
        """현재 실행 중인 모든 ROS 노드 목록 조회"""
        try:
            import rosnode
            nodes = rosnode.get_node_names()
            return {
                "success": True,
                "nodes": nodes,
                "count": len(nodes)
            }
        except Exception as e:
            return {"error": str(e)}

    def get_ros_info(self) -> dict:
        """ROS 시스템 정보 조회"""
        try:
            return {
                "success": True,
                "ros_version": "ROS 1",
                "ros_distro": os.getenv("ROS_DISTRO", "unknown"),
                "ros_master_uri": os.getenv("ROS_MASTER_URI", "unknown"),
                "node_name": self.node_name
            }
        except Exception as e:
            return {"error": str(e)}

    def check_ros_connection(self) -> dict:
        """ROS 연결 상태 확인"""
        try:
            master_uri = os.getenv("ROS_MASTER_URI", "unknown")
            is_connected = not rospy.is_shutdown()
            return {
                "success": True,
                "ros_initialized": is_connected,
                "ros_master_uri": master_uri,
                "node_name": self.node_name
            }
        except Exception as e:
            return {"error": str(e)}

    # ============== G1 전용 메서드 (서브클래스에서 오버라이드) ==============

    def stand_up(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 stand_up을 지원하지 않습니다"}

    def sit_down(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 sit_down을 지원하지 않습니다"}

    def walk(self, direction: str = "forward", speed: float = 0.5, duration: float = 2.0) -> dict:
        return {"error": f"{self.__class__.__name__}는 walk를 지원하지 않습니다"}

    def turn(self, angle: float = 90.0, direction: str = "left") -> dict:
        return {"error": f"{self.__class__.__name__}는 turn을 지원하지 않습니다"}

    def walk_to(self, x: float, y: float) -> dict:
        return {"error": f"{self.__class__.__name__}는 walk_to를 지원하지 않습니다"}

    def set_gait_mode(self, mode: int) -> dict:
        return {"error": f"{self.__class__.__name__}는 set_gait_mode를 지원하지 않습니다"}

    def get_joint_states(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 get_joint_states를 지원하지 않습니다"}

    def get_imu_data(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 get_imu_data를 지원하지 않습니다"}

    def get_robot_status(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 get_robot_status를 지원하지 않습니다"}

    def wave_hand(self, hand: str = "right") -> dict:
        return {"error": f"{self.__class__.__name__}는 wave_hand를 지원하지 않습니다"}

    def bow(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 bow를 지원하지 않습니다"}

    def set_damping_mode(self) -> dict:
        return {"error": f"{self.__class__.__name__}는 set_damping_mode를 지원하지 않습니다"}
