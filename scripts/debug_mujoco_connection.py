#!/usr/bin/env python3
"""
Mujoco 시뮬레이터와의 DDS 연결 디버그 스크립트

1. LowState 수신 확인
2. LowCmd 전송 테스트
"""

import time
import sys

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

# 전역 변수
low_state = None
state_received_count = 0

def low_state_callback(msg: LowState_):
    global low_state, state_received_count
    low_state = msg
    state_received_count += 1

def main():
    global low_state, state_received_count

    print("=" * 60)
    print("Mujoco DDS 연결 디버그")
    print("=" * 60)
    print("\n[INFO] Mujoco 시뮬레이터가 실행 중이어야 합니다!")
    print("[INFO] Domain ID: 1, Interface: lo")
    print()

    # DDS 초기화
    print("[1/4] DDS 초기화 중...")
    ChannelFactoryInitialize(1, "lo")
    print("      ✓ DDS 초기화 완료")

    # Publisher 생성
    print("[2/4] LowCmd Publisher 생성 중...")
    low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_pub.Init()
    print("      ✓ Publisher 생성 완료 (토픽: rt/lowcmd)")

    # Subscriber 생성
    print("[3/4] LowState Subscriber 생성 중...")
    low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
    low_state_sub.Init(low_state_callback, 10)
    print("      ✓ Subscriber 생성 완료 (토픽: rt/lowstate)")

    # LowState 수신 대기
    print("\n[4/4] LowState 수신 대기 중...")
    print("      (5초 동안 대기)")

    for i in range(50):
        time.sleep(0.1)
        if low_state is not None:
            print(f"\n      ✓ LowState 수신 완료! (수신 횟수: {state_received_count})")
            break
    else:
        print("\n      ✗ LowState 수신 실패!")
        print("      → Mujoco 시뮬레이터가 실행 중인지 확인하세요")
        print("      → Domain ID와 Interface가 일치하는지 확인하세요")
        return

    # 현재 관절 상태 출력
    print("\n" + "=" * 60)
    print("현재 관절 상태 (처음 6개)")
    print("=" * 60)
    for i in range(min(6, 29)):
        q = low_state.motor_state[i].q
        dq = low_state.motor_state[i].dq
        print(f"  Joint {i}: q={q:.4f} rad, dq={dq:.4f} rad/s")

    # LowCmd 전송 테스트
    print("\n" + "=" * 60)
    print("LowCmd 전송 테스트")
    print("=" * 60)

    crc = CRC()
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = 0

    # 현재 위치를 유지하면서 작은 kp로 테스트
    print("\n[테스트] 현재 위치 유지 (kp=10, kd=1) - 3초간")

    start_time = time.time()
    cmd_count = 0

    while (time.time() - start_time) < 3.0:
        # 현재 위치 읽기
        for i in range(29):
            if low_state:
                current_q = low_state.motor_state[i].q
            else:
                current_q = 0.0

            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = float(current_q)  # 현재 위치 유지
            cmd.motor_cmd[i].kp = 10.0  # 작은 kp
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 1.0
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        low_cmd_pub.Write(cmd)
        cmd_count += 1

        time.sleep(0.02)  # 50Hz

    print(f"      ✓ LowCmd 전송 완료 (총 {cmd_count}개 명령)")
    print(f"      → 수신된 LowState 수: {state_received_count}")

    # 최종 상태 확인
    print("\n" + "=" * 60)
    print("최종 관절 상태 (처음 6개)")
    print("=" * 60)
    for i in range(min(6, 29)):
        q = low_state.motor_state[i].q
        dq = low_state.motor_state[i].dq
        print(f"  Joint {i}: q={q:.4f} rad, dq={dq:.4f} rad/s")

    print("\n[완료] 디버그 종료")
    print("\n만약 로봇이 움직이지 않았다면:")
    print("  1. Mujoco 뷰어에서 로봇이 보이는지 확인")
    print("  2. 시뮬레이터 콘솔에 에러가 있는지 확인")
    print("  3. kp, kd 값을 높여서 다시 테스트")

if __name__ == "__main__":
    main()
