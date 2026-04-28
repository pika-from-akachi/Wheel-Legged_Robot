"""Interactive real-time control with keyboard.
Keys:
  Q/W: hip  +/-   A/S: knee  +/-   Z/X: wheel +/-
  Space: reset     ESC: exit
"""

import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
ROBOT_XML = HERE / "robot.xml"

model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
data = mujoco.MjData(model)

# Initial standing pose
data.qpos[2] = 0.31  # torso height
t_hip, t_knee, t_wheel = 0.0, -0.2, 0.0

def reset():
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.31
    global t_hip, t_knee, t_wheel
    t_hip, t_knee, t_wheel = 0.0, -0.2, 0.0

# Actuator IDs: 0=left_hip, 1=left_knee, 2=left_wheel,
#              3=right_hip, 4=right_knee, 5=right_wheel
ACT = {"lh": 0, "lk": 1, "lw": 2, "rh": 3, "rk": 4, "rw": 5}

print("=== 交互式轮足控制 ===")
print("Q/W → 髋关节 +/-   A/S → 膝关节 +/-   Z/X → 轮子 +/-")
print("Space → 复位   ESC → 退出\n")

def key_callback(key: int) -> None:
    global t_hip, t_knee, t_wheel
    if key == 32:  # Space
        reset(); return
    h = 0.05; k = 0.05; w = 1.0
    if key == 113: t_hip = min(1.5, t_hip + h)       # q
    elif key == 119: t_hip = max(-1.5, t_hip - h)     # w
    elif key == 97: t_knee = min(0.0, t_knee + k)     # a
    elif key == 115: t_knee = max(-2.0, t_knee - k)   # s
    elif key == 122: t_wheel = min(30, t_wheel + w)   # z
    elif key == 120: t_wheel = max(-30, t_wheel - w)  # x

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.distance = 1.5
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0, 0, 0.15])

    while viewer.is_running():
        # Use built-in position actuators (kp=200, kv=10 from robot.xml)
        data.ctrl[ACT["lh"]] = t_hip
        data.ctrl[ACT["lk"]] = t_knee
        data.ctrl[ACT["lw"]] = t_wheel  # motor = torque
        data.ctrl[ACT["rh"]] = t_hip
        data.ctrl[ACT["rk"]] = t_knee
        data.ctrl[ACT["rw"]] = t_wheel

        mujoco.mj_step(model, data)
        viewer.sync()
