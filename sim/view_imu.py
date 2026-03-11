from pathlib import Path
import time
import mujoco

MODEL_PATH = Path("unitree_ros/robots/g1_description/g1_29dof.xml")

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

for step in range(200):
    mujoco.mj_step(model, data)

print("=== sensor data ===")
for i in range(model.nsensor):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
    adr = model.sensor_adr[i]
    dim = model.sensor_dim[i]
    values = data.sensordata[adr:adr+dim]
    print(f"{name}: {values}")