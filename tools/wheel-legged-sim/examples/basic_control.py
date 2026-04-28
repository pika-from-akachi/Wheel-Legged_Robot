"""Simple control example: make the robot crouch and stand."""

import json
import time
from pathlib import Path

import numpy as np

from wheel_leg_sim.simulation import WheelLegSimulation

sim = WheelLegSimulation()

print("=== Step 1: Stand up from crouch ===")
for t in np.arange(0, 0.6, 0.02):
    hip = 0.0
    knee = -1.0 + t * 1.5  # straighten knee over 0.6 s
    sim.step(np.array([hip, knee, 0.0, hip, knee, 0.0]), nsteps=10)
    if int(t / 0.1) != int((t - 0.02) / 0.1):
        s = sim.get_state()
        print(f"  t={s['time']:.2f}s  height={s['torso_height']:.3f}m  "
              f"L_knee={s['joint_positions']['left_knee']:.2f}")

print("\n=== Step 2: Render frame ===")
from PIL import Image as PILImage
img = PILImage.fromarray(sim.render())
img.save("frame_stand.png")
print("Saved frame_stand.png")

print("\n=== Step 3: Sway side to side ===")
for direction in [1, -1, 1]:
    for t in np.arange(0, 0.3, 0.02):
        hip = direction * 0.3
        sim.step(np.array([hip, -0.3, 0.0, -hip, -0.3, 0.0]), nsteps=10)
    s = sim.get_state()
    print(f"  sway -> t={s['time']:.2f}s  "
          f"L_hip={s['joint_positions']['left_hip']:.2f}  "
          f"R_hip={s['joint_positions']['right_hip']:.2f}")

print("\nDone. Close the simulation.")
sim.close()
