import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

HERE = Path(__file__).parent
ROBOT_XML = HERE.parent / "robot.xml"


class WheelLegSimulation:
    def __init__(self, xml_path: str | Path = ROBOT_XML):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self._renderer: mujoco.Renderer | None = None

        self.joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_names.append(name)

        self.actuator_names = []
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.actuator_names.append(name)

        self.imu_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "imu"
        )

        self.sensor_map: dict[str, int] = {}
        for i in range(self.model.nsensor):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            if name:
                self.sensor_map[name] = i

    def step(self, ctrl: np.ndarray | None = None, nsteps: int = 1) -> None:
        if ctrl is not None:
            self.data.ctrl[:] = ctrl
        for _ in range(nsteps):
            mujoco.mj_step(self.model, self.data)

    def get_state(self) -> dict[str, Any]:
        mujoco.mj_forward(self.model, self.data)

        joint_positions = {}
        joint_velocities = {}
        for name in self.joint_names:
            jnt = self.model.joint(name)
            jnt_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            qposadr = self.model.jnt_qposadr[jnt_id]
            dofadr = self.model.jnt_dofadr[jnt_id]
            joint_positions[name] = float(self.data.qpos[qposadr])
            joint_velocities[name] = float(self.data.qvel[dofadr])

        return {
            "time": float(self.data.time),
            "position": {
                "x": float(self.data.qpos[0]),
                "y": float(self.data.qpos[1]),
                "z": float(self.data.qpos[2]),
            },
            "torso_height": float(self.data.xpos[1][2]),
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "sensors": self._read_sensors(),
        }

    def render(self, width: int = 640, height: int = 480) -> np.ndarray:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)

        camera = mujoco.MjvCamera()
        camera.distance = 1.5
        camera.azimuth = 90
        camera.elevation = -20
        camera.lookat = np.array([0, 0, 0.2])

        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def reset(
        self, joint_positions: dict[str, float] | None = None
    ) -> None:
        mujoco.mj_resetData(self.model, self.data)
        if joint_positions:
            for name, pos in joint_positions.items():
                jnt_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                )
                if jnt_id >= 0:
                    qposadr = self.model.jnt_qposadr[jnt_id]
                    self.data.qpos[qposadr] = pos
        mujoco.mj_forward(self.model, self.data)

    def close(self) -> None:
        if self._renderer:
            self._renderer.close()
            self._renderer = None

    def _read_sensors(self) -> dict[str, list[float]]:
        data = {}
        for name, sid in self.sensor_map.items():
            addr = self.model.sensor_adr[sid]
            dim = self.model.sensor_dim[sid]
            data[name] = self.data.sensordata[addr : addr + dim].tolist()
        return data
