"""Real-time MuJoCo viewer for wheel-legged robot simulation.
Opens an interactive 3D window. Mouse: orbit (left), pan (right), scroll (zoom)."""

import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path

HERE = Path(__file__).parent
ROBOT_XML = HERE / "robot.xml"

model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
data = mujoco.MjData(model)

# Raise torso above ground
data.qpos[2] = 0.32

print("=== 实时仿真查看器 ===")
print("鼠标左键: 旋转视角")
print("鼠标右键: 平移")
print("滚轮: 缩放")
print("按 ESC 退出\n")

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 1.5
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0, 0, 0.15])

    step = 0
    while viewer.is_running():
        # Simple PD controller: hold standing pose
        kp, kv = 200, 10
        target_hip = 0.0
        target_knee = -0.15

        for joint_name, target in [("left_hip", target_hip), ("left_knee", target_knee),
                                    ("right_hip", target_hip), ("right_knee", target_knee)]:
            jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            dofadr = model.jnt_dofadr[jnt_id]
            qposadr = model.jnt_qposadr[jnt_id]
            pos_err = target - data.qpos[qposadr]
            vel = data.qvel[dofadr]
            # Torque = kp * pos_err - kv * vel (gravity compensation would be added)
            data.ctrl[dofadr] = kp * pos_err - kv * vel

        mujoco.mj_step(model, data)
        viewer.sync()

        step += 1
