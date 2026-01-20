#!/usr/bin/env python3
"""
가장 간단한 Mujoco 제어 테스트
- unitree_sdk2_python 예제와 동일한 방식으로 테스트
"""

import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

G1_NUM_MOTOR = 29

# PD 게인 (unitree 예제와 동일)
Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40, 40, 40, 40,   # left arm
    40, 40, 40, 40, 40, 40, 40    # right arm
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # left arm
    1, 1, 1, 1, 1, 1, 1   # right arm
]


class SimpleController:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  # 50Hz (시뮬레이터와 맞춤)
        self.duration_ = 3.0

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_state_received = False
        self.crc = CRC()

        self.cmd_count = 0
        self.state_count = 0

    def init(self):
        print("[Init] Publisher 생성...")
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()

        print("[Init] Subscriber 생성...")
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self.low_state_handler, 10)

        print("[Init] 완료")

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg
        self.state_count += 1
        if not self.first_state_received:
            self.first_state_received = True
            print(f"[Callback] 첫 LowState 수신!")

    def start(self):
        print("\n[Start] LowState 대기 중...")
        timeout = 5.0
        start = time.time()
        while not self.first_state_received:
            if time.time() - start > timeout:
                print("[Error] LowState 수신 시간 초과!")
                return
            time.sleep(0.1)

        print(f"[Start] LowState 수신 완료 (총 {self.state_count}개)")
        print("\n[Start] 제어 시작 - 현재 위치 유지 (3초)")

        # RecurrentThread 대신 직접 루프
        start_time = time.time()
        while (time.time() - start_time) < self.duration_:
            self.control_step()
            time.sleep(self.control_dt_)

        print(f"\n[완료] 전송된 명령: {self.cmd_count}개")
        print(f"[완료] 수신된 상태: {self.state_count}개")

    def control_step(self):
        if self.low_state is None:
            return

        self.time_ += self.control_dt_

        # 모든 관절을 현재 위치로 유지
        for i in range(G1_NUM_MOTOR):
            current_q = self.low_state.motor_state[i].q

            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = 0
            self.low_cmd.motor_cmd[i].mode = 1  # Enable
            self.low_cmd.motor_cmd[i].q = float(current_q)
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = float(Kp[i])
            self.low_cmd.motor_cmd[i].kd = float(Kd[i])
            self.low_cmd.motor_cmd[i].tau = 0.0

        # CRC 계산 및 전송
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_pub.Write(self.low_cmd)
        self.cmd_count += 1

        # 진행 상황 출력
        if self.cmd_count % 10 == 0:
            q0 = self.low_state.motor_state[0].q
            print(f"\r  명령 {self.cmd_count}개 전송됨, Joint0 q={q0:.4f}", end="", flush=True)


def main():
    print("=" * 60)
    print("간단한 Mujoco G1 제어 테스트")
    print("=" * 60)
    print("\n주의: Mujoco 시뮬레이터가 먼저 실행 중이어야 합니다!")
    print("  cd ~/unitree_ws/src/unitree_mujoco/simulate_python")
    print("  python3 unitree_mujoco.py")
    print()

    # DDS 초기화 (시뮬레이터와 동일: domain=1, interface=eth0)
    print("[DDS] 초기화 중... (domain=1, interface=eth0)")
    ChannelFactoryInitialize(1, "eth0")
    print("[DDS] 초기화 완료")

    controller = SimpleController()
    controller.init()
    controller.start()

    # 최종 상태 출력
    if controller.low_state:
        print("\n\n최종 관절 상태:")
        for i in range(6):
            q = controller.low_state.motor_state[i].q
            print(f"  Joint {i}: q={q:.4f} rad")


if __name__ == "__main__":
    main()
