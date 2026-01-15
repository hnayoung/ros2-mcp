import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative, Spawn, Kill
from turtlesim.msg import Pose

from .base_adapter import BaseRobotAdapter


class TurtlesimAdapter(BaseRobotAdapter):
    """Turtlesim 로봇 제어를 위한 어댑터"""

    # Turtlesim 환경 상수
    WINDOW_MIN = 0.0
    WINDOW_MAX = 11.0
    DEFAULT_TURTLE = 'turtle1'

    def __init__(self, turtle_name: str = None):
        self.turtle_name = turtle_name or self.DEFAULT_TURTLE
        cmd_vel_topic = f'/{self.turtle_name}/cmd_vel'

        super().__init__(
            node_name='turtlesim_mcp_adapter',
            cmd_vel_topic=cmd_vel_topic
        )

        # Turtlesim 전용 파라미터
        self.max_linear_speed = 2.0
        self.max_angular_speed = 2.0
        self.rate_hz = 10

        # Pose 토픽 구독
        self.pose_topic = f'/{self.turtle_name}/pose'
        self.current_pose = None
        self.pose_sub = self.node.create_subscription(
            Pose,
            self.pose_topic,
            self._pose_callback,
            QoSProfile(depth=10)
        )

        # Turtlesim 전용 서비스 클라이언트
        self.set_pen_client = self.node.create_client(
            SetPen, f'/{self.turtle_name}/set_pen'
        )
        self.teleport_abs_client = self.node.create_client(
            TeleportAbsolute, f'/{self.turtle_name}/teleport_absolute'
        )
        self.teleport_rel_client = self.node.create_client(
            TeleportRelative, f'/{self.turtle_name}/teleport_relative'
        )
        self.spawn_client = self.node.create_client(Spawn, '/spawn')
        self.kill_client = self.node.create_client(Kill, '/kill')

    def _pose_callback(self, msg: Pose):
        """현재 위치 업데이트"""
        self.current_pose = msg

    # ============== BaseRobotAdapter 필수 구현 ==============

    def get_pose(self) -> dict:
        """현재 거북이 위치/방향 조회"""
        self._spin_once()
        if self.current_pose:
            return {
                "success": True,
                "x": self.current_pose.x,
                "y": self.current_pose.y,
                "theta": self.current_pose.theta,
                "linear_velocity": self.current_pose.linear_velocity,
                "angular_velocity": self.current_pose.angular_velocity
            }
        return {"error": "Pose not available yet"}

    # ============== Turtlesim 전용 메서드 ==============

    def set_pen(self, r: int = 255, g: int = 255, b: int = 255,
                width: int = 3, off: bool = False) -> dict:
        """펜 설정 (색상, 두께, on/off)"""
        if not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            return {"error": "set_pen service not available"}

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = int(off)

        future = self.set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        return {
            "success": True,
            "action": "set_pen",
            "color": {"r": r, "g": g, "b": b},
            "width": width,
            "off": off
        }

    def pen_up(self) -> dict:
        """펜 들기 (그리기 중지)"""
        return self.set_pen(off=True)

    def pen_down(self, r: int = 255, g: int = 255, b: int = 255, width: int = 3) -> dict:
        """펜 내리기 (그리기 시작)"""
        return self.set_pen(r=r, g=g, b=b, width=width, off=False)

    def teleport_absolute(self, x: float, y: float, theta: float = 0.0) -> dict:
        """절대 좌표로 순간이동"""
        x = max(self.WINDOW_MIN, min(x, self.WINDOW_MAX))
        y = max(self.WINDOW_MIN, min(y, self.WINDOW_MAX))

        if not self.teleport_abs_client.wait_for_service(timeout_sec=1.0):
            return {"error": "teleport_absolute service not available"}

        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)

        future = self.teleport_abs_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        return {
            "success": True,
            "action": "teleport_absolute",
            "x": x,
            "y": y,
            "theta": theta
        }

    def teleport_relative(self, linear: float, angular: float) -> dict:
        """상대 좌표로 순간이동"""
        if not self.teleport_rel_client.wait_for_service(timeout_sec=1.0):
            return {"error": "teleport_relative service not available"}

        request = TeleportRelative.Request()
        request.linear = float(linear)
        request.angular = float(angular)

        future = self.teleport_rel_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        return {
            "success": True,
            "action": "teleport_relative",
            "linear": linear,
            "angular": angular
        }

    def go_to_center(self) -> dict:
        """화면 중앙으로 이동"""
        center = (self.WINDOW_MAX - self.WINDOW_MIN) / 2
        return self.teleport_absolute(center, center, 0.0)

    def spawn_turtle(self, x: float = 5.5, y: float = 5.5,
                     theta: float = 0.0, name: str = "") -> dict:
        """새 거북이 생성"""
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            return {"error": "spawn service not available"}

        request = Spawn.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        request.name = name

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.result():
            return {
                "success": True,
                "action": "spawn",
                "name": future.result().name,
                "x": x,
                "y": y,
                "theta": theta
            }
        return {"error": "Failed to spawn turtle"}

    def kill_turtle(self, name: str) -> dict:
        """거북이 제거"""
        if not self.kill_client.wait_for_service(timeout_sec=1.0):
            return {"error": "kill service not available"}

        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        return {
            "success": True,
            "action": "kill",
            "name": name
        }

    # ============== BaseRobotAdapter 오버라이드 (Turtlesim 맞춤) ==============

    def move_forward(self, speed: float = 1.0, duration: float = 1.0) -> dict:
        """
        거북이를 앞으로 직진 이동

        Args:
            speed (float): 선속도 (m/s), 기본값 1.0
            duration (float): 이동 시간 (초), 기본값 1.0

        Returns:
            dict: 이동 결과 (시작/종료 위치 포함)
        """
        self._spin_once()
        start_pose = self._get_pose_dict()

        result = super().move_forward(speed, duration)

        self._spin_once()
        end_pose = self._get_pose_dict()

        result["turtle_name"] = self.turtle_name
        result["start_pose"] = start_pose
        result["end_pose"] = end_pose
        return result

    def move_backward(self, speed: float = 1.0, duration: float = 1.0) -> dict:
        """
        거북이를 뒤로 후진 이동

        Args:
            speed (float): 선속도 (m/s), 기본값 1.0
            duration (float): 이동 시간 (초), 기본값 1.0

        Returns:
            dict: 이동 결과 (시작/종료 위치 포함)
        """
        self._spin_once()
        start_pose = self._get_pose_dict()

        result = super().move_backward(speed, duration)

        self._spin_once()
        end_pose = self._get_pose_dict()

        result["turtle_name"] = self.turtle_name
        result["start_pose"] = start_pose
        result["end_pose"] = end_pose
        return result

    def rotate(self, angle: float = 1.57, angular_speed: float = 1.0, direction: str = "left") -> dict:
        """
        거북이를 제자리에서 회전

        Args:
            angle (float): 회전 각도 (라디안), 기본값 π/2 (90도)
            angular_speed (float): 회전 속도 (rad/s), 기본값 1.0
            direction (str): 회전 방향 ("left" 또는 "right"), 기본값 "left"

        Returns:
            dict: 회전 결과 (시작/종료 방향 포함)
        """
        self._spin_once()
        start_pose = self._get_pose_dict()

        result = super().rotate(angle, angular_speed, direction)

        self._spin_once()
        end_pose = self._get_pose_dict()

        result["turtle_name"] = self.turtle_name
        result["start_pose"] = start_pose
        result["end_pose"] = end_pose
        return result

    def stop(self) -> dict:
        """거북이 정지"""
        result = super().stop()
        result["turtle_name"] = self.turtle_name
        return result

    def _get_pose_dict(self) -> dict:
        """현재 위치를 dict로 반환 (내부용)"""
        if self.current_pose:
            return {
                "x": round(self.current_pose.x, 3),
                "y": round(self.current_pose.y, 3),
                "theta": round(self.current_pose.theta, 3)
            }
        return None
