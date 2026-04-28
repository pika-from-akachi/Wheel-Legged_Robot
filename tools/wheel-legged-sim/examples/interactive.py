"""Interactive real-time control with keyboard.
Keys:
  Q/W: left hip  +/-     A/S: left knee  +/-     Z/X: left wheel  +/-
  E/R: right hip +/-     D/F: right knee +/-     C/V: right wheel +/-
  Space: reset
"""

import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path

HERE = Path(__file__).parent
ROBOT_XML = HERE / "robot.xml"

model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
data = mujoco.MjData(model)

# Initial pose
data.qpos[2] = 0.32
targets = {"hip": 0.0, "knee": -0.15, "wheel": 0.0}
LEFT, RIGHT = 0, 1

def reset():
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.32
    targets.update({"hip": 0.0, "knee": -0.15, "wheel": 0.0})

print("=== 交互式轮足控制 ===")
print("Q/W → 左髋 +/-\tE/R → 右髋 +/-")
print("A/S → 左膝 +/-\tD/F → 右膝 +/-")
print("Z/X → 左轮 +/-\tC/V → 右轮 +/-")
print("Space → 复位\tESC → 退出\n")

kp, kv = 300, 15

def key_callback(key: int) -> None:
    step = 0.05
    if key == 32:  # Space
        reset()
    # Left leg
    if key == 113:  # q
        targets["hip"] = min(1.5, targets["hip"] + step)
    elif key == 119:  # w
        targets["hip"] = max(-1.5, targets["hip"] - step)
    elif key == 97:   # a
        targets["knee"] = min(0.0, targets["knee"] + step)
    elif key == 115:  # s
        targets["knee"] = max(-2.0, targets["knee"] - step)
    elif key == 122:  # z
        targets["wheel"] = min(30, targets["wheel"] + 1)
    elif key == 120:  # x
        targets["wheel"] = max(-30, targets["wheel"] - 1)
    # Right leg
    elif key == 101:  # e
        targets["hip"] = min(1.5, targets["hip"] + step)
    elif key == 114:  # r
        targets["hip"] = max(-1.5, targets["hip"] - step)
    elif key == 100:  # d
        targets["knee"] = min(0.0, targets["knee"] + step)
    elif key == 102:  # f
        targets["knee"] = max(-2.0, targets["knee"] - step)
    elif key == 99:   # c
        targets["wheel"] = min(30, targets["wheel"] + 1)
    elif key == 118:  # v
        targets["wheel"] = max(-30, targets["wheel"] - 1)

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.distance = 1.5
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0, 0, 0.15])

    while viewer.is_running():
        joints = [
            ("left_hip", targets["hip"]), ("left_knee", targets["knee"]),
            ("left_wheel", targets["wheel"]),
            ("right_hip", targets["hip"]), ("right_knee", targets["knee"]),
            ("right_wheel", targets["wheel"]),
        ]
        for name, target in joints:
            jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            dofadr = model.jnt_dofadr[jnt_id]
            qposadr = model.jnt_qposadr[jnt_id]
            pos_err = target - data.qpos[qposadr]
            vel = data.qvel[dofadr]
            data.ctrl[dofadr] = kp * pos_err - kv * vel

        mujoco.mj_step(model, data)
        viewer.sync()
