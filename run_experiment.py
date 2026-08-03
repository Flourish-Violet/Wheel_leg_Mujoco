from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

from controllers import make_controller
from environment import LegWheelRobot


LOG_FIELDS = [
    "time",
    "roll",
    "pitch",
    "yaw",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "wheel_vel_r",
    "wheel_vel_l",
    "joint_rf",
    "joint_rr",
    "joint_lf",
    "joint_lr",
    "ctrl_rf",
    "ctrl_rr",
    "ctrl_lf",
    "ctrl_lr",
    "ctrl_rw",
    "ctrl_lw",
    "x",
    "dx",
]


def flatten_observation(obs: dict) -> dict:
    gyro = obs["gyro"]
    wheel_vel = obs["wheel_vel"]
    joint_pos = obs["joint_pos"]
    ctrl = obs["ctrl"]
    return {
        "time": obs["time"],
        "roll": obs["roll"],
        "pitch": obs["pitch"],
        "yaw": obs["yaw"],
        "gyro_x": gyro[0],
        "gyro_y": gyro[1],
        "gyro_z": gyro[2],
        "wheel_vel_r": wheel_vel[0],
        "wheel_vel_l": wheel_vel[1],
        "joint_rf": joint_pos[0],
        "joint_rr": joint_pos[1],
        "joint_lf": joint_pos[2],
        "joint_lr": joint_pos[3],
        "ctrl_rf": ctrl[0],
        "ctrl_rr": ctrl[1],
        "ctrl_lf": ctrl[2],
        "ctrl_lr": ctrl[3],
        "ctrl_rw": ctrl[4],
        "ctrl_lw": ctrl[5],
        "x": obs["x"],
        "dx": obs["dx"],
    }


def save_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def save_plot(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    t = np.array([r["time"] for r in rows])
    pitch = np.array([r["pitch"] for r in rows])
    gyro_y = np.array([r["gyro_y"] for r in rows])
    ctrl_rw = np.array([r["ctrl_rw"] for r in rows])
    ctrl_lw = np.array([r["ctrl_lw"] for r in rows])

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(t, pitch)
    axes[0].set_ylabel("pitch rad")
    axes[0].grid(True)
    axes[1].plot(t, gyro_y)
    axes[1].set_ylabel("gyro_y rad/s")
    axes[1].grid(True)
    axes[2].plot(t, ctrl_rw, label="right wheel")
    axes[2].plot(t, ctrl_lw, label="left wheel")
    axes[2].set_ylabel("wheel torque")
    axes[2].set_xlabel("time s")
    axes[2].legend()
    axes[2].grid(True)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run(controller_name: str, duration: float, out_dir: Path, wheel_torque: float, kp: float, kd: float) -> None:
    robot = LegWheelRobot("MJCF/env.xml", viewer=False)
    controller = make_controller(controller_name, wheel_torque=wheel_torque, kp=kp, kd=kd)
    rows = []
    steps = int(duration / robot.model.opt.timestep)

    try:
        robot.sensor_read_data()
        for _ in range(steps):
            obs = robot.sensor_read_data()
            torque = controller.compute(obs, robot.data.time)
            robot.apply_torque(torque)
            robot.step()
            rows.append(flatten_observation(robot.get_observation()))
    finally:
        robot.close()

    save_csv(rows, out_dir / "log.csv")
    save_plot(rows, out_dir / "summary.png")

    max_abs_pitch = max(abs(r["pitch"]) for r in rows) if rows else 0.0
    max_abs_ctrl = max(max(abs(r[k]) for k in ["ctrl_rf", "ctrl_rr", "ctrl_lf", "ctrl_lr", "ctrl_rw", "ctrl_lw"]) for r in rows) if rows else 0.0
    print(f"controller={controller_name}")
    print(f"steps={len(rows)} duration={duration}s timestep={robot.model.opt.timestep}")
    print(f"max_abs_pitch={max_abs_pitch:.6f} rad")
    print(f"max_abs_ctrl={max_abs_ctrl:.6f}")
    print(f"csv={out_dir / 'log.csv'}")
    print(f"plot={out_dir / 'summary.png'}")


def inspect_model() -> None:
    model = mujoco.MjModel.from_xml_path("MJCF/env.xml")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} nsensor={model.nsensor} nsensordata={model.nsensordata}")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        lo, hi = model.actuator_ctrlrange[i]
        print(f"ctrl[{i}] {name}: range=[{lo:g}, {hi:g}]")
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        print(f"sensor[{i}] {name}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run repeatable wheel-leg MuJoCo control experiments.")
    parser.add_argument("--controller", choices=["zero", "wheel_step", "pitch_pd"], default="zero")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--out", type=Path, default=Path("runs") / "latest")
    parser.add_argument("--wheel-torque", type=float, default=0.4)
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--kd", type=float, default=0.05)
    parser.add_argument("--inspect", action="store_true")
    args = parser.parse_args()

    if args.inspect:
        inspect_model()
        return

    run(args.controller, args.duration, args.out, args.wheel_torque, args.kp, args.kd)


if __name__ == "__main__":
    main()
