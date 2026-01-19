#!/usr/bin/env python3
"""
DDS 통신 디버깅 스크립트
시뮬레이터가 실행 중일 때 실행하세요.
"""

import time
import sys

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.utils.crc import CRC
    print("[OK] unitree_sdk2py 임포트 성공")
except ImportError as e:
    print(f"[ERROR] unitree_sdk2py 임포트 실패: {e}")
    sys.exit(1)


class DDSDebugger:
    def __init__(self):
        self.low_state = None
        self.low_state_count = 0
        self.crc = CRC()

    def low_state_callback(self, msg):
        self.low_state = msg
        self.low_state_count += 1

    def run(self):
        print("\n[INFO] DDS 초기화 중...")
        ChannelFactoryInitialize(1, "lo")

        # LowState 구독
        print("[INFO] rt/lowstate 구독 시작...")
        low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        low_state_sub.Init(self.low_state_callback, 10)

        # LowCmd 퍼블리셔
        print("[INFO] rt/lowcmd 퍼블리셔 초기화...")
        low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        low_cmd_pub.Init()

        print("\n" + "=" * 50)
        print("DDS 통신 테스트 시작")
        print("시뮬레이터가 실행 중이어야 합니다!")
        print("=" * 50)

        # 3초 동안 LowState 수신 대기
        print("\n[1] LowState 수신 테스트 (3초)...")
        time.sleep(3)

        if self.low_state_count > 0:
            print(f"  [OK] LowState 메시지 수신: {self.low_state_count}개")
            if self.low_state:
                print(f"  - motor_state[0].q: {self.low_state.motor_state[0].q:.4f}")
                print(f"  - motor_state[0].dq: {self.low_state.motor_state[0].dq:.4f}")
        else:
            print("  [FAIL] LowState 메시지를 수신하지 못했습니다!")
            print("  -> 시뮬레이터(unitree_mujoco.py)가 실행 중인지 확인하세요")
            return

        # LowCmd 전송 테스트
        print("\n[2] LowCmd 전송 테스트...")
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0

        # 현재 관절 위치 유지 명령 (kp, kd 설정)
        for i in range(29):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = self.low_state.motor_state[i].q if self.low_state else 0.0
            cmd.motor_cmd[i].kp = 50.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = self.crc.Crc(cmd)

        # 1초 동안 명령 전송
        print("  명령 전송 중 (1초)...")
        start = time.time()
        count = 0
        while time.time() - start < 1.0:
            low_cmd_pub.Write(cmd)
            count += 1
            time.sleep(0.002)  # 500Hz

        print(f"  [OK] {count}개의 LowCmd 메시지 전송")

        # 결과 확인
        print("\n[3] 최종 상태 확인...")
        time.sleep(0.5)

        final_count = self.low_state_count
        print(f"  총 LowState 수신: {final_count}개")

        if self.low_state:
            print(f"  현재 관절 위치 (처음 6개):")
            for i in range(6):
                print(f"    motor[{i}].q = {self.low_state.motor_state[i].q:.4f}")

        print("\n" + "=" * 50)
        print("테스트 완료")
        print("=" * 50)


if __name__ == "__main__":
    debugger = DDSDebugger()
    debugger.run()
