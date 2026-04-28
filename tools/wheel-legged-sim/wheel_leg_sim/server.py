import json
from io import BytesIO

import numpy as np
from mcp.server.fastmcp import FastMCP, Image
from PIL import Image as PILImage

from wheel_leg_sim.simulation import WheelLegSimulation

sim = WheelLegSimulation()
mcp = FastMCP("Wheel-Legged Robot Simulator")

JOINT_ORDER = ["left_hip", "left_knee", "left_wheel",
               "right_hip", "right_knee", "right_wheel"]
CTRL_DOC = (
    "left_hip: left hip position target (rad). "
    "left_knee: left knee position target (rad). "
    "left_wheel: left wheel torque. "
    "right_hip: right hip position target (rad). "
    "right_knee: right knee position target (rad). "
    "right_wheel: right wheel torque. "
    "Hip and knee are PD position-controlled; wheels are torque-controlled."
)


@mcp.tool(description=f"Apply motor commands and step simulation. {CTRL_DOC}")
def step_simulation(
    left_hip: float = 0.0,
    left_knee: float = -1.0,
    left_wheel: float = 0.0,
    right_hip: float = 0.0,
    right_knee: float = -1.0,
    right_wheel: float = 0.0,
    duration: float = 0.1,
) -> str:
    nsteps = max(1, int(duration / sim.model.opt.timestep))
    ctrl = np.array([left_hip, left_knee, left_wheel,
                     right_hip, right_knee, right_wheel])
    sim.step(ctrl, nsteps)
    state = sim.get_state()
    state["steps_taken"] = nsteps
    return json.dumps(state, indent=2)


@mcp.tool(description="Get current joint angles, velocities, IMU, and body position")
def get_state() -> str:
    return json.dumps(sim.get_state(), indent=2, default=str)


@mcp.tool(description="Render the current simulation as a webp image")
def render() -> Image:
    pixels = sim.render()
    pil = PILImage.fromarray(pixels)
    buf = BytesIO()
    pil.save(buf, format="WEBP", quality=85)
    return Image(data=buf.getvalue(), format="webp")


@mcp.tool(description=(
    "Reset simulation. Optionally set initial hip/knee positions (rad). "
    "Wheels always reset to zero."
))
def reset_simulation(
    left_hip: float = 0.0,
    left_knee: float = -1.0,
    right_hip: float = 0.0,
    right_knee: float = -1.0,
) -> str:
    sim.reset({
        "left_hip": left_hip,
        "left_knee": left_knee,
        "right_hip": right_hip,
        "right_knee": right_knee,
    })
    return json.dumps(sim.get_state(), indent=2, default=str)


@mcp.tool(description="Get joint names, actuator names, and model parameters")
def get_joint_info() -> str:
    return json.dumps({
        "joints": sim.joint_names,
        "actuators": sim.actuator_names,
        "timestep": sim.model.opt.timestep,
        "gravity": sim.model.opt.gravity.tolist(),
    }, indent=2)


@mcp.tool(description="Step the simulation with zero control (free-fall / passive dynamics)")
def step_passive(duration: float = 0.1) -> str:
    nsteps = max(1, int(duration / sim.model.opt.timestep))
    sim.step(None, nsteps)
    state = sim.get_state()
    state["steps_taken"] = nsteps
    return json.dumps(state, indent=2, default=str)


if __name__ == "__main__":
    mcp.run(transport="stdio")
