from pathlib import Path
import math
import threading

import mujoco
import mujoco.viewer

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from unitree_hg.msg import HandCmd, HandState, LowCmd, LowState, MotorState

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


MODEL_PATH = (
    Path(__file__).resolve().parent
    / "unitree_ros/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.xml"
)

LOG_EVERY_N_STEPS = 120
LOG_JOINTS = (
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "waist_yaw_joint",
)

DEFAULT_POSE = {
    # 몸통이 XML에서 고정된 환경 기준 기본 자세(허리/팔/다리 정렬)
    "left_hip_pitch_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.0,
    "left_ankle_pitch_joint": 0.0,
    "left_ankle_roll_joint": 0.0,
    "right_hip_pitch_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.0,
    "right_ankle_pitch_joint": 0.0,
    "right_ankle_roll_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "waist_roll_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.0,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.0,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
    # Inspire hand 관절값(모두 0으로 시작)
    "L_thumb_proximal_yaw_joint": 0.0,
    "L_thumb_proximal_pitch_joint": 0.0,
    "L_index_proximal_joint": 0.0,
    "L_middle_proximal_joint": 0.0,
    "L_ring_proximal_joint": 0.0,
    "L_pinky_proximal_joint": 0.0,
    "R_thumb_proximal_yaw_joint": 0.0,
    "R_thumb_proximal_pitch_joint": 0.0,
    "R_index_proximal_joint": 0.0,
    "R_middle_proximal_joint": 0.0,
    "R_ring_proximal_joint": 0.0,
    "R_pinky_proximal_joint": 0.0,
}

PAUSE_KEYCODE = 32
is_paused = False

ROS2_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
LEFT_HAND_JOINT_NAMES = [
    "L_thumb_proximal_yaw_joint",
    "L_thumb_proximal_pitch_joint",
    "L_index_proximal_joint",
    "L_middle_proximal_joint",
    "L_ring_proximal_joint",
    "L_pinky_proximal_joint",
]
RIGHT_HAND_JOINT_NAMES = [
    "R_thumb_proximal_yaw_joint",
    "R_thumb_proximal_pitch_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_ring_proximal_joint",
    "R_pinky_proximal_joint",
]


class ROS2Bridge:
    def __init__(self, model):
        self.model = model
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("g1_mujoco_bridge", context=self.context)

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.lowcmd_sub = self.node.create_subscription(
            LowCmd, "/lowcmd", self._lowcmd_callback, qos_best_effort
        )
        self.lowstate_pub = self.node.create_publisher(
            LowState, "/lowstate", qos_best_effort
        )
        self.left_hand_cmd_sub = self.node.create_subscription(
            HandCmd, "/inspire/left/cmd", self._left_hand_cmd_callback, qos_best_effort
        )
        self.right_hand_cmd_sub = self.node.create_subscription(
            HandCmd, "/inspire/right/cmd", self._right_hand_cmd_callback, qos_best_effort
        )
        self.left_hand_state_pub = self.node.create_publisher(
            HandState, "/lf/inspire/left/state", qos_best_effort
        )
        self.right_hand_state_pub = self.node.create_publisher(
            HandState, "/lf/inspire/right/state", qos_best_effort
        )

        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            name="g1_mujoco_bridge_executor",
            daemon=True,
        )
        self.executor_thread.start()

        self.lock = threading.Lock()
        self.latest_cmd = None
        self.latest_left_hand_cmd = None
        self.latest_right_hand_cmd = None
        self.latest_mode_machine = 0

        self.joint_ids = []
        self.actuator_ids = []
        for joint_name in ROS2_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
            self.joint_ids.append(joint_id)
            self.actuator_ids.append(actuator_id)
        self.left_hand_joint_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            for joint_name in LEFT_HAND_JOINT_NAMES
        ]
        self.right_hand_joint_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            for joint_name in RIGHT_HAND_JOINT_NAMES
        ]

        print("[ROS2Bridge] Subscribed: /lowcmd")
        print("[ROS2Bridge] Subscribed: /inspire/left/cmd")
        print("[ROS2Bridge] Subscribed: /inspire/right/cmd")
        print("[ROS2Bridge] Publishing: /lowstate")
        print("[ROS2Bridge] Publishing: /lf/inspire/left/state")
        print("[ROS2Bridge] Publishing: /lf/inspire/right/state")

    def _lowcmd_callback(self, msg):
        with self.lock:
            self.latest_cmd = msg
            self.latest_mode_machine = int(msg.mode_machine)

    def _left_hand_cmd_callback(self, msg):
        with self.lock:
            self.latest_left_hand_cmd = msg

    def _right_hand_cmd_callback(self, msg):
        with self.lock:
            self.latest_right_hand_cmd = msg

    def apply_lowcmd(self, model, data):
        with self.lock:
            cmd = self.latest_cmd
            left_hand_cmd = self.latest_left_hand_cmd
            right_hand_cmd = self.latest_right_hand_cmd

        log_overrides = {}
        data.qfrc_applied[:] = 0.0
        if model.nu > 0:
            data.ctrl[:] = 0.0

        if cmd is not None:
            for index, joint_name in enumerate(ROS2_JOINT_NAMES):
                actuator_id = self.actuator_ids[index]
                joint_id = self.joint_ids[index]
                if joint_id < 0:
                    continue

                motor_cmd = cmd.motor_cmd[index]
                qpos_adr = model.jnt_qposadr[joint_id]
                dof_adr = model.jnt_dofadr[joint_id]

                raw_ctrl = (
                    float(motor_cmd.tau)
                    + float(motor_cmd.kp) * (float(motor_cmd.q) - data.qpos[qpos_adr])
                    + float(motor_cmd.kd) * (float(motor_cmd.dq) - data.qvel[dof_adr])
                )
                clamped_ctrl, _, _ = clamp_ctrl_to_actuator_limit(model, joint_id, raw_ctrl)
                if actuator_id >= 0:
                    data.ctrl[actuator_id] = clamped_ctrl
                else:
                    data.qfrc_applied[dof_adr] = clamped_ctrl
                log_overrides[joint_name] = raw_ctrl

        self._apply_handcmd(data, left_hand_cmd, LEFT_HAND_JOINT_NAMES, self.left_hand_joint_ids)
        self._apply_handcmd(data, right_hand_cmd, RIGHT_HAND_JOINT_NAMES, self.right_hand_joint_ids)

        return log_overrides

    def _apply_handcmd(self, data, cmd, joint_names, joint_ids):
        if cmd is None:
            return

        for index, joint_id in enumerate(joint_ids):
            if joint_id < 0 or index >= len(cmd.motor_cmd):
                continue

            motor_cmd = cmd.motor_cmd[index]
            qpos_adr = self.model.jnt_qposadr[joint_id]
            dof_adr = self.model.jnt_dofadr[joint_id]
            raw_ctrl = (
                float(motor_cmd.tau)
                + float(motor_cmd.kp) * (float(motor_cmd.q) - data.qpos[qpos_adr])
                + float(motor_cmd.kd) * (float(motor_cmd.dq) - data.qvel[dof_adr])
            )
            clamped_ctrl, _, _ = clamp_ctrl_to_actuator_limit(self.model, joint_id, raw_ctrl)
            data.qfrc_applied[dof_adr] = clamped_ctrl

    def publish_state(self, model, data):
        low_state = LowState()
        low_state.mode_pr = 0
        low_state.mode_machine = self.latest_mode_machine

        base_pose = get_base_pose(model, data)
        if base_pose is not None:
            pos, quat = base_pose
            pelvis_roll, pelvis_pitch, pelvis_yaw = quat_to_euler_wxyz(quat)
            free_base_joint_id = get_free_base_joint_id(model)
            qvel_adr = model.jnt_dofadr[free_base_joint_id]
            angular_velocity = data.qvel[qvel_adr + 3 : qvel_adr + 6]
            low_state.imu_state.quaternion = [float(v) for v in quat]
            low_state.imu_state.rpy = [pelvis_roll, pelvis_pitch, pelvis_yaw]
            low_state.imu_state.gyroscope = [float(v) for v in angular_velocity]
            low_state.imu_state.accelerometer = [0.0, 0.0, 0.0]
        else:
            low_state.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
            low_state.imu_state.rpy = [0.0, 0.0, 0.0]
            low_state.imu_state.gyroscope = [0.0, 0.0, 0.0]
            low_state.imu_state.accelerometer = [0.0, 0.0, 0.0]

        for index, joint_name in enumerate(ROS2_JOINT_NAMES):
            joint_id = self.joint_ids[index]
            actuator_id = self.actuator_ids[index]
            if joint_id < 0:
                continue

            qpos_adr = model.jnt_qposadr[joint_id]
            dof_adr = model.jnt_dofadr[joint_id]
            low_state.motor_state[index].q = float(data.qpos[qpos_adr])
            low_state.motor_state[index].dq = float(data.qvel[dof_adr])
            low_state.motor_state[index].tau_est = (
                float(data.ctrl[actuator_id]) if actuator_id >= 0 else float(data.qfrc_applied[dof_adr])
            )

        self.lowstate_pub.publish(low_state)
        self.left_hand_state_pub.publish(
            self._build_hand_state(data, LEFT_HAND_JOINT_NAMES, self.left_hand_joint_ids)
        )
        self.right_hand_state_pub.publish(
            self._build_hand_state(data, RIGHT_HAND_JOINT_NAMES, self.right_hand_joint_ids)
        )

    def _build_hand_state(self, data, joint_names, joint_ids):
        hand_state = HandState()
        hand_state.motor_state = []
        for joint_id in joint_ids:
            if joint_id < 0:
                continue
            qpos_adr = self.model.jnt_qposadr[joint_id]
            dof_adr = self.model.jnt_dofadr[joint_id]
            motor_state = MotorState()
            motor_state.q = float(data.qpos[qpos_adr])
            motor_state.dq = float(data.qvel[dof_adr])
            motor_state.ddq = 0.0
            motor_state.tau_est = float(data.qfrc_applied[dof_adr])
            hand_state.motor_state.append(motor_state)
        return hand_state

    def shutdown(self):
        self.executor.shutdown()
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=1.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


def get_free_base_joint_id(model):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
    if joint_id == -1:
        return None
    if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        return None
    return joint_id


def get_base_pose(model, data):
    free_base_joint_id = get_free_base_joint_id(model)
    if free_base_joint_id is None:
        return None

    qpos_adr = model.jnt_qposadr[free_base_joint_id]
    pos = data.qpos[qpos_adr : qpos_adr + 3]
    quat = data.qpos[qpos_adr + 3 : qpos_adr + 7]
    return pos, quat


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


def initialize_pose(model, data):
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qfrc_applied[:] = 0.0
    if model.nu > 0:
        data.ctrl[:] = 0.0

    for joint_name, target in DEFAULT_POSE.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            continue
        qpos_adr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = target

    base_pose = get_base_pose(model, data)
    if base_pose is not None:
        pos, _ = base_pose
        pos[2] = max(pos[2], 0.82)
    mujoco.mj_forward(model, data)


def clamp_ctrl_to_actuator_limit(model, joint_id, ctrl):
    if joint_id < 0:
        return ctrl, None, None
    if model.jnt_actfrclimited[joint_id]:
        lower, upper = model.jnt_actfrcrange[joint_id]
        clamped_ctrl = min(max(ctrl, lower), upper)
        return clamped_ctrl, lower, upper
    return ctrl, None, None


def apply_standing_controller(model, data):
    log_rows = []
    lowcmd_raw = ros2_bridge.apply_lowcmd(model, data) if ros2_bridge is not None else None

    for actuator_id in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        joint_id = model.actuator_trnid[actuator_id][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        dof_adr = model.jnt_dofadr[joint_id]

        if lowcmd_raw is not None:
            raw_ctrl = lowcmd_raw.get(actuator_name, 0.0)
        else:
            raw_ctrl = 0.0
            data.ctrl[actuator_id] = 0.0
        _, lower, upper = clamp_ctrl_to_actuator_limit(model, joint_id, data.ctrl[actuator_id])

        if actuator_name in LOG_JOINTS:
            log_rows.append(
                (
                    actuator_name,
                    data.qpos[qpos_adr],
                    data.qvel[dof_adr],
                    raw_ctrl,
                    data.ctrl[actuator_id],
                    lower,
                    upper,
                )
            )

    if model.nu == 0:
        for joint_name in LOG_JOINTS:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                continue
            qpos_adr = model.jnt_qposadr[joint_id]
            dof_adr = model.jnt_dofadr[joint_id]
            raw_ctrl = lowcmd_raw.get(joint_name, 0.0) if lowcmd_raw is not None else 0.0
            _, lower, upper = clamp_ctrl_to_actuator_limit(model, joint_id, data.qfrc_applied[dof_adr])
            log_rows.append(
                (
                    joint_name,
                    data.qpos[qpos_adr],
                    data.qvel[dof_adr],
                    raw_ctrl,
                    data.qfrc_applied[dof_adr],
                    lower,
                    upper,
                )
            )

    return log_rows


def log_controller_state(step_count, data, log_rows):
    base_pose = get_base_pose(model, data)
    if base_pose is not None:
        pos, quat = base_pose
        pelvis_roll, pelvis_pitch, pelvis_yaw = quat_to_euler_wxyz(quat)
        print(
            (
                f"[step {step_count:05d}] "
                f"roll={math.degrees(pelvis_roll):7.2f} deg "
                f"pitch={math.degrees(pelvis_pitch):7.2f} deg "
                f"yaw={math.degrees(pelvis_yaw):7.2f} deg "
                f"z={pos[2]:.3f}"
            )
        )
    else:
        print(f"[step {step_count:05d}] fixed_base")
    for actuator_name, qpos, qvel, raw_ctrl, ctrl, lower, upper in log_rows:
        limit_text = ""
        if lower is not None and upper is not None:
            limit_text = f" range=[{lower:6.1f},{upper:6.1f}]"
        print(
            (
                f"[joint] {actuator_name:24s} "
                f"qpos={qpos:7.3f} "
                f"qvel={qvel:7.3f} "
                f"raw={raw_ctrl:8.3f} "
                f"ctrl={ctrl:8.3f}"
                f"{limit_text}"
            )
        )
    print()


def configure_camera(viewer):
    viewer.cam.lookat[:] = (0.0, 0.0, 0.78)
    viewer.cam.distance = 3.2
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -18.0


def key_callback(keycode):
    global is_paused
    if keycode == PAUSE_KEYCODE:
        is_paused = not is_paused
        state = "paused" if is_paused else "running"
        print(f"Simulation {state}")


print(f"Loading model: {MODEL_PATH}")
model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)
ros2_bridge = ROS2Bridge(model) if ROS2_AVAILABLE else None
if not ROS2_AVAILABLE:
    print(
        "[WARN] ROS2 bridge disabled. Source /home/na0/workspace/ros2-mcp/ros2/unitree_ros2/setup_local.sh before running."
    )
initialize_pose(model, data)

try:
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        configure_camera(viewer)
        step_count = 0
        while viewer.is_running():
            if not is_paused:
                log_rows = apply_standing_controller(model, data)
                mujoco.mj_step(model, data)
                if ros2_bridge is not None:
                    ros2_bridge.publish_state(model, data)
                step_count += 1
                if step_count % LOG_EVERY_N_STEPS == 0:
                    log_controller_state(step_count, data, log_rows)
            viewer.sync()
finally:
    if ros2_bridge is not None:
        ros2_bridge.shutdown()
