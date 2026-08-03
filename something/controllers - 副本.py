from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class TorqueCommand:
    """Six actuator torque commands in MuJoCo ctrl order."""

    right_front: float = 0.0
    right_rear: float = 0.0
    left_front: float = 0.0
    left_rear: float = 0.0
    right_wheel: float = 0.0
    left_wheel: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array(
            [
                self.right_front,
                self.right_rear,
                self.left_front,
                self.left_rear,
                self.right_wheel,
                self.left_wheel,
            ],
            dtype=float,
        )


class Controller(Protocol):
    name: str

    def compute(self, obs: dict, t: float) -> np.ndarray:
        """Return six torque commands in MuJoCo ctrl order."""


class ZeroTorqueController:
    name = "zero"

    def compute(self, obs: dict, t: float) -> np.ndarray:
        return TorqueCommand().as_array()


class WheelStepController:
    name = "wheel_step"

    def __init__(self, wheel_torque: float = 0.4, start_time: float = 0.5):
        self.wheel_torque = float(wheel_torque)
        self.start_time = float(start_time)

    def compute(self, obs: dict, t: float) -> np.ndarray:
        if t < self.start_time:
            return TorqueCommand().as_array()
        return TorqueCommand(
            right_wheel=self.wheel_torque,
            left_wheel=self.wheel_torque,
        ).as_array()


class PitchPDController:
    """Simple pitch baseline for later replacement by VMC/LQR/RL."""

    name = "pitch_pd"

    def __init__(self, kp: float = 1.0, kd: float = 0.05, target_pitch: float = 0.0):
        self.kp = float(kp)
        self.kd = float(kd)
        self.target_pitch = float(target_pitch)

    def compute(self, obs: dict, t: float) -> np.ndarray:
        pitch = float(obs.get("pitch", 0.0))
        gyro_y = float(obs.get("gyro", np.zeros(3))[1])
        wheel = self.kp * (self.target_pitch - pitch) - self.kd * gyro_y
        wheel = float(np.clip(wheel, -4.0, 4.0))
        return TorqueCommand(right_wheel=wheel, left_wheel=wheel).as_array()


def make_controller(name: str, **kwargs) -> Controller:
    if name == "zero":
        return ZeroTorqueController()
    if name == "wheel_step":
        return WheelStepController(wheel_torque=kwargs.get("wheel_torque", 0.4))
    if name == "pitch_pd":
        return PitchPDController(kp=kwargs.get("kp", 1.0), kd=kwargs.get("kd", 0.05))
    raise ValueError(f"Unknown controller: {name}")
