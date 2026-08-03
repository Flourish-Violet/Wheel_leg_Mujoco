from __future__ import annotations

import ctypes
from pathlib import Path

import numpy as np

from environment import LegWheelRobot
from keyboard import KeyboardController


ROOT = Path(__file__).resolve().parent
CPP_VMC_DLL = ROOT / "cpp_control" / "build-ninja" / "vmc_bridge.dll"


class CppVmcController:
    def __init__(self, dll_path: Path = CPP_VMC_DLL):
        if not dll_path.exists():
            raise FileNotFoundError(
                f"C++ VMC DLL not found: {dll_path}\n"
                "Build it first: cmake --build E:\\Mujoco\\wheel_leg_mujoco\\cpp_control\\build-ninja"
            )

        self.lib = ctypes.CDLL(str(dll_path))
        double_p = ctypes.POINTER(ctypes.c_double)
        self.lib.vmc_compute_c.argtypes = [
            ctypes.c_double,
            double_p,
            double_p,
            double_p,
            double_p,
            double_p,
            double_p,
            ctypes.c_double,
            double_p,
        ]
        self.lib.vmc_compute_c.restype = None

    def compute(self, robot: LegWheelRobot, dt: float) -> np.ndarray:
        quat = np.asarray(robot.orien, dtype=np.float64)
        euler = np.asarray(robot.euler, dtype=np.float64)
        gyro = np.asarray(robot.gyro, dtype=np.float64)
        joint_pos = np.asarray(robot.joint_pos, dtype=np.float64)
        wheel_pos = np.asarray([robot.right_wheel_pos, robot.left_wheel_pos], dtype=np.float64)
        wheel_vel = np.asarray(robot.wheel_vel, dtype=np.float64)
        out = np.zeros(6, dtype=np.float64)

        self.lib.vmc_compute_c(
            ctypes.c_double(float(robot.data.time)),
            quat.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            euler.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            gyro.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            joint_pos.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            wheel_pos.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            wheel_vel.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ctypes.c_double(float(dt)),
            out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        )
        return out


def apply_ctrl(robot: LegWheelRobot, ctrl: np.ndarray) -> None:
    if hasattr(robot, "apply_torque"):
        robot.apply_torque(ctrl)
        return

    robot.joint_torque = ctrl[:4].tolist()
    robot.wheel_torque = ctrl[4:6].tolist()
    robot.actuator_set_torque()


def main() -> None:
    robot = LegWheelRobot("MJCF/env.xml", viewer=True)
    controller = CppVmcController()
    keyboard = KeyboardController()

    step_count = 0
    sensor_div = 1
    control_div = 4
    keyboard_div = 20

    try:
        while True:
            step_count += 1
            robot.step()

            if step_count % sensor_div == 0:
                robot.sensor_read_data()

            if step_count % control_div == 0 and len(robot.joint_pos) == 4:
                ctrl = controller.compute(robot, robot.model.opt.timestep * control_div)
                apply_ctrl(robot, ctrl)

            if step_count % keyboard_div == 0:
                _ = keyboard.get_command()
    finally:
        robot.close()


if __name__ == "__main__":
    main()
