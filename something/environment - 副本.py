from __future__ import annotations

import math
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from caculation import orientation2euler


class LegWheelRobot:
    """Wheel-leg robot simulation wrapper.

    The actuator order is fixed by `MJCF/robot.xml`:
    [right_front, right_rear, left_front, left_rear, right_wheel, left_wheel].
    """

    def __init__(self, model_path: str = "MJCF/env.xml", viewer: bool = True):
        self.model_path = str(Path(model_path))
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if viewer else None

        self.sensor_T = float(self.model.opt.timestep)
        self.sensor_f = 1.0 / self.sensor_T
        self.wheel_r = 0.77

        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)
        self.orien = np.array([1.0, 0.0, 0.0, 0.0])
        self.euler = np.zeros(3)
        self.joint_pos = np.zeros(4)
        self.wheel_vel = np.zeros(2)

        self.x = 0.0
        self.d_x = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_wheel_pos = 0.0
        self.last_right_wheel_pos = 0.0

        self.wheel_torque = [0.0, 0.0]
        self.joint_torque = [0.0, 0.0, 0.0, 0.0]

        if self.viewer is not None:
            print("MuJoCo viewer started. Press ESC in the viewer to exit.")

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def sensor_read_data(self) -> dict:
        mujoco.mj_forward(self.model, self.data)

        self.orien = self.data.sensor("orientation").data.copy()
        self.euler = np.asarray(orientation2euler(self.orien), dtype=float)
        self.gyro = self.data.sensor("gyro").data.copy()

        self.right_wheel_pos = float(self.data.sensor("Right_Wheel_pos").data.copy()[0])
        self.left_wheel_pos = float(self.data.sensor("Left_Wheel_pos").data.copy()[0])
        self.wheel_vel[0] = round((self.right_wheel_pos - self.last_right_wheel_pos) * self.sensor_f, 3)
        self.wheel_vel[1] = -round((self.left_wheel_pos - self.last_left_wheel_pos) * self.sensor_f, 3)
        self.last_right_wheel_pos = self.right_wheel_pos
        self.last_left_wheel_pos = self.left_wheel_pos

        self.d_x = (self.wheel_vel[0] + self.wheel_vel[1]) * 0.5 * 2.0 * math.pi * self.wheel_r / 60.0
        self.x += self.d_x * self.sensor_T

        right_front_pos = float(self.data.sensor("Right_front_joint_pos").data.copy()[0]) + 0.027
        right_rear_pos = float(self.data.sensor("Right_rear_joint_pos").data.copy()[0]) + 1.3
        left_front_pos = float(self.data.sensor("Left_front_joint_pos").data.copy()[0]) + 0.003
        left_rear_pos = float(self.data.sensor("Left_rear_joint_pos").data.copy()[0]) - 1.3
        self.joint_pos = np.array([right_front_pos, right_rear_pos, left_front_pos, left_rear_pos], dtype=float)
        return self.get_observation()

    def get_observation(self) -> dict:
        return {
            "time": float(self.data.time),
            "orientation": self.orien.copy(),
            "euler": self.euler.copy(),
            "roll": float(self.euler[0]),
            "pitch": float(self.euler[1]),
            "yaw": float(self.euler[2]),
            "gyro": self.gyro.copy(),
            "joint_pos": self.joint_pos.copy(),
            "wheel_vel": self.wheel_vel.copy(),
            "x": float(self.x),
            "dx": float(self.d_x),
            "ctrl": self.data.ctrl.copy(),
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
        }

    def apply_torque(self, torque) -> None:
        torque = np.asarray(torque, dtype=float)
        if torque.shape != (6,):
            raise ValueError(f"Expected 6 torque commands, got shape {torque.shape}")
        self.data.ctrl[:] = np.clip(torque, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        self.joint_torque = self.data.ctrl[:4].tolist()
        self.wheel_torque = self.data.ctrl[4:6].tolist()

    def actuator_set_torque(self) -> None:
        self.apply_torque([*self.joint_torque, *self.wheel_torque])

    def set_joint_positions(self, joint_angles):
        joint_names = ["jAG", "jGH", "jIO", "jOP"]
        for i, name in enumerate(joint_names):
            if i >= len(joint_angles):
                break
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id != -1:
                qpos_addr = self.model.jnt_qposadr[joint_id]
                qvel_addr = self.model.jnt_dofadr[joint_id]
                self.data.qpos[qpos_addr] = joint_angles[i]
                self.data.qvel[qvel_addr] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.x = 0.0
        self.d_x = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_wheel_pos = 0.0
        self.last_right_wheel_pos = 0.0
        self.wheel_vel = np.zeros(2)
        self.joint_pos = np.zeros(4)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
