#!/usr/bin/env python3
"""
G1 팔 제어 테스트 (MuJoCo 시뮬레이터용)
원본: unitree_sdk2_python/example/g1/high_level/g1_arm5_sdk_dds_example.py

시뮬레이터 설정:
- Domain ID: 1
- Interface: lo
- Topic: rt/lowcmd (시뮬레이터는 rt/arm_sdk 대신 rt/lowcmd 사용)
"""

import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

kPi = 3.141592654
kPi_2 = 1.57079632


class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


class G1ArmController:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  # 50Hz
        self.duration_ = 3.0
        self.kp = 60.0
        self.kd = 1.5

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update = False
        self.crc = CRC()
        self.done = False

        # 목표 자세: 양팔 들어올리기
        self.target_pos = [
            0.0,      kPi_2,  0.0,    kPi_2,  0.0,   # 왼팔: shoulder pitch/roll/yaw, elbow, wrist
            0.0,     -kPi_2,  0.0,    kPi_2,  0.0,   # 오른팔
            0.0,      0.0,    0.0                     # 허리
        ]

        # 제어할 관절들
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch
        ]

    def init(self):
        # 시뮬레이터용 설정: rt/lowcmd 토픽 사용
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.low_state_handler, 10)

        print("[init] Publisher/Subscriber 초기화 완료")

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg
        if not self.first_update:
            self.first_update = True
            print("[callback] LowState 첫 수신 완료")

    def start(self):
        print("[start] LowState 대기 중...")
        while not self.first_update:
            time.sleep(0.1)

        print("[start] 팔 제어 시작!")
        self.control_thread = RecurrentThread(
            interval=self.control_dt_, target=self.control_loop, name="arm_control"
        )
        self.control_thread.Start()

    def control_loop(self):
        self.time_ += self.control_dt_

        # Stage 1 (0~3초): 현재 위치 유지하며 제어권 확보
        if self.time_ < self.duration_:
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            print(f"\r[Stage 1] 초기화 중... {ratio*100:.0f}%", end="", flush=True)

            for i, joint in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].q = self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp * ratio
                self.low_cmd.motor_cmd[joint].kd = self.kd

        # Stage 2 (3~9초): 팔 들어올리기
        elif self.time_ < self.duration_ * 3:
            ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)
            print(f"\r[Stage 2] 팔 들어올리기... {ratio*100:.0f}%", end="", flush=True)

            for i, joint in enumerate(self.arm_joints):
                current_q = self.low_state.motor_state[joint].q
                target_q = self.target_pos[i]
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * current_q + ratio * target_q
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        # Stage 3 (9~18초): 팔 내리기
        elif self.time_ < self.duration_ * 6:
            ratio = np.clip((self.time_ - self.duration_ * 3) / (self.duration_ * 3), 0.0, 1.0)
            print(f"\r[Stage 3] 팔 내리기... {ratio*100:.0f}%", end="", flush=True)

            for i, joint in enumerate(self.arm_joints):
                current_q = self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * current_q
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        else:
            print("\n[완료] 팔 제어 종료")
            self.done = True
            return

        # CRC 계산 및 명령 전송
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


if __name__ == "__main__":
    print("=" * 50)
    print("G1 팔 제어 테스트 (MuJoCo 시뮬레이터용)")
    print("=" * 50)
    print("\n주의: unitree_mujoco 시뮬레이터가 실행 중이어야 합니다!")
    print("실행 방법: cd ~/unitree_ws/src/unitree_mujoco/simulate_python && python3 unitree_mujoco.py\n")

    # 시뮬레이터 설정: Domain ID=1, Interface=lo
    ChannelFactoryInitialize(1, "lo")
    print("[DDS] 초기화 완료 (domain=1, interface=lo)")

    controller = G1ArmController()
    controller.init()
    controller.start()

    # 완료까지 대기
    while not controller.done:
        time.sleep(0.5)

    print("\n테스트 완료!")
