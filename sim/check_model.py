from pathlib import Path
import mujoco

MODEL_PATH = Path(__file__).resolve().parent / "unitree_ros/robots/g1_description/g1_29dof.urdf"
print("Loading:", MODEL_PATH)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print("Model loaded successfully")
print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)