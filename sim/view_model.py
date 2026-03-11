from pathlib import Path
import math

import mujoco
import mujoco.viewer

MODEL_PATH = (
    Path(__file__).resolve().parent
    / "unitree_ros/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.xml"
)


STAND_POSE = {
    "left_hip_pitch_joint": 0,
    "left_knee_joint": 0,
    "left_ankle_pitch_joint": 0,
    "right_hip_pitch_joint": 0,
    "right_knee_joint": 0,
    "right_ankle_pitch_joint": 0,
    "waist_pitch_joint": 0,
    "left_shoulder_roll_joint": 0,
    "right_shoulder_roll_joint": 0,
    "left_elbow_joint": 0,
    "right_elbow_joint": 0,
}

PELVIS_NAME = "pelvis"


def quat_to_euler_wxyz(quat):
    w, x, y, z = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def joint_gains(actuator_name):
    if any(part in actuator_name for part in ("hip_", "knee", "ankle")):
        return 220.0, 18.0
    if "waist" in actuator_name:
        return 120.0, 12.0
    if any(part in actuator_name for part in ("shoulder", "elbow", "wrist")):
        return 40.0, 4.0
    return 8.0, 0.8


def target_for_actuator(actuator_name, pelvis_roll, pelvis_pitch, roll_rate, pitch_rate):
    target = STAND_POSE.get(actuator_name, 0.0)

    if actuator_name in ("left_ankle_pitch_joint", "right_ankle_pitch_joint"):
        target += -0.80 * pelvis_pitch - 0.10 * pitch_rate
    elif actuator_name in ("left_hip_pitch_joint", "right_hip_pitch_joint"):
        target += -0.45 * pelvis_pitch - 0.06 * pitch_rate
    elif actuator_name == "left_ankle_roll_joint":
        target += -0.70 * pelvis_roll - 0.08 * roll_rate
    elif actuator_name == "right_ankle_roll_joint":
        target += -0.70 * pelvis_roll - 0.08 * roll_rate
    elif actuator_name == "left_hip_roll_joint":
        target += -0.35 * pelvis_roll - 0.05 * roll_rate
    elif actuator_name == "right_hip_roll_joint":
        target += -0.35 * pelvis_roll - 0.05 * roll_rate

    return target


def initialize_pose(model, data):
    for joint_name, target in STAND_POSE.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_adr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = target

    data.qpos[2] = 0.72
    mujoco.mj_forward(model, data)

def apply_standing_controller(model, data):
    pelvis_roll, pelvis_pitch, _ = quat_to_euler_wxyz(data.qpos[3:7])
    roll_rate = data.qvel[3]
    pitch_rate = data.qvel[4]

    for actuator_id in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        joint_id = model.actuator_trnid[actuator_id][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        dof_adr = model.jnt_dofadr[joint_id]

        kp, kd = joint_gains(actuator_name)
        target = target_for_actuator(
            actuator_name,
            pelvis_roll,
            pelvis_pitch,
            roll_rate,
            pitch_rate,
        )
        position_error = target - data.qpos[qpos_adr]
        velocity_error = -data.qvel[dof_adr]

        # qfrc_bias adds gravity/Coriolis compensation for the actuated dof.
        data.ctrl[actuator_id] = (
            data.qfrc_bias[dof_adr] + kp * position_error + kd * velocity_error
        )


print(f"Loading model: {MODEL_PATH}")
model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)
initialize_pose(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        apply_standing_controller(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
