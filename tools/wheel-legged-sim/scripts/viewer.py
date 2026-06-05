#!/usr/bin/env python3
"""Launch MuJoCo interactive viewer for the wheel-legged robot with 刀盾 model."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import mujoco.viewer
from wheel_leg_sim.simulation import WheelLegSimulation

sim = WheelLegSimulation()
sim.reset({"left_hip": 0.0, "left_knee": -1.0, "right_hip": 0.0, "right_knee": -1.0})
mujoco.mj_forward(sim.model, sim.data)

ctrl = np.array([0.0, -1.0, 0.0, 0.0, -1.0, 0.0])

print("=" * 50)
print("MuJoCo 3D 交互界面")
print("左键拖动: 旋转视角")
print("滚轮: 缩放")
print("右键拖动: 平移")
print()
print("提示: 初始姿态已冻结，关闭窗口可退出")
print("=" * 50)

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    viewer.cam.distance = 1.2
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -15
    viewer.cam.lookat = np.array([0.0, 0.0, 0.35])

    # Show initial state first (before any stepping)
    viewer.sync()

    # Then start simulation loop
    while viewer.is_running():
        sim.step(ctrl, 5)
        viewer.sync()
