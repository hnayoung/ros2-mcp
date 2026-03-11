from pathlib import Path
import mujoco

MODEL_PATH = Path("unitree_ros/robots/g1_description/g1_29dof.xml")

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

print("nsensor =", model.nsensor)

for i in range(model.nsensor):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
    adr = model.sensor_adr[i]
    dim = model.sensor_dim[i]
    sensor_type = model.sensor_type[i]
    print(f"{i:2d}  name={name}, adr={adr}, dim={dim}, type={sensor_type}")