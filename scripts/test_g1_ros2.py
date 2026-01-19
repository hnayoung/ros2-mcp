#!/usr/bin/env python3
"""
G1 ROS 2 어댑터 테스트 스크립트
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

def main():
    print("=" * 50)
    print("G1 ROS 2 어댑터 테스트")
    print("=" * 50)

    # ROS 2 사용 가능 여부 확인
    try:
        import rclpy
        print("[OK] rclpy 임포트 성공")
    except ImportError:
        print("[ERROR] rclpy를 찾을 수 없습니다.")
        print("  - ROS 2 환경을 source 했는지 확인하세요:")
        print("    source /opt/ros/humble/setup.bash")
        return

    # G1 어댑터 임포트
    try:
        from ros_mcp.server.adapters.g1_adapter import G1Adapter, UNITREE_SDK_AVAILABLE
        print("[OK] G1Adapter 임포트 성공")
        print(f"[INFO] Unitree SDK 사용 가능: {UNITREE_SDK_AVAILABLE}")
    except Exception as e:
        print(f"[ERROR] G1Adapter 임포트 실패: {e}")
        import traceback
        traceback.print_exc()
        return

    # 어댑터 생성
    try:
        print("\n[INFO] G1 어댑터 초기화 중...")
        adapter = G1Adapter()
        print("[OK] G1 어댑터 초기화 성공")
    except Exception as e:
        print(f"[ERROR] G1 어댑터 초기화 실패: {e}")
        import traceback
        traceback.print_exc()
        return

    # 상태 확인
    print("\n" + "-" * 50)
    print("연결 상태:")
    print("-" * 50)

    ros_status = adapter.check_ros_connection()
    print(f"  ROS 연결: {ros_status}")

    robot_status = adapter.get_robot_status()
    print(f"  로봇 상태: {robot_status}")

    pose = adapter.get_pose()
    print(f"  현재 위치: {pose}")

    # 토픽 목록
    print("\n" + "-" * 50)
    print("ROS 2 토픽:")
    print("-" * 50)
    topics = adapter.get_topics()
    if topics.get("success"):
        for t in topics.get("topics", [])[:10]:
            types_str = ", ".join(t['types']) if t['types'] else "unknown"
            print(f"  - {t['name']} [{types_str}]")
        if topics.get("count", 0) > 10:
            print(f"  ... 외 {topics['count'] - 10}개")
    else:
        print(f"  토픽 조회 실패: {topics.get('error')}")

    # ROS 정보
    print("\n" + "-" * 50)
    print("ROS 2 정보:")
    print("-" * 50)
    ros_info = adapter.get_ros_info()
    for key, value in ros_info.items():
        print(f"  {key}: {value}")

    print("\n" + "=" * 50)
    print("테스트 완료")
    print("=" * 50)

    # 종료
    adapter.shutdown()


if __name__ == "__main__":
    main()
