# Wheel Leg MuJoCo

这是一个基于 [MuJoCo](https://mujoco.readthedocs.io/) 的轮腿机器人仿真与控制实验仓库，主要用于：

1. 验证轮腿机构的动力学与运动学模型
2. 统一管理 Python 控制器与实验流程
3. 通过 C++ 桥接做更高性能的控制实验

当前仓库的主线思路是先在 MuJoCo 中把模型、传感器和基础控制器跑通，再逐步扩展到 VMC、PID、LQR 或强化学习方法。

## 目录结构

```text
wheel_leg_mujoco/
├── MJCF/                  # MuJoCo 模型、网格资源、场景 XML
│   ├── env.xml
│   └── robot.xml
├── cpp_control/           # C++ VMC 桥接与可视化示例
├── environment.py         # MuJoCo 机器人封装
├── controllers.py         # Python 基线控制器
├── run_experiment.py      # 批量实验与日志输出
├── Simulation.py          # 交互式仿真，加载 C++ 桥接
├── keyboard.py            # 键盘输入辅助
├── VMC.py                 # Python 侧 VMC 逻辑
├── caculation.py          # 四元数与欧拉角转换工具
├── requirements.txt       # Python 依赖
└── runs/                  # 实验输出目录
```

## 环境要求

- Windows
- Python 3.11，推荐使用 Conda 管理环境
- `mujoco==3.4.0`
- 如果要编译 `cpp_control`，还需要 CMake 和支持 C++17 的编译器

`requirements.txt` 当前依赖如下：

```text
mujoco==3.4.0
numpy
matplotlib
pynput
```

## 快速开始

### 1. 创建 Python 环境

```powershell
conda create -n mujoco311 python=3.11
conda activate mujoco311
cd E:\Mujoco\wheel_leg_mujoco
python -m pip install -r requirements.txt
```

### 2. 检查模型接口

```powershell
python run_experiment.py --inspect
```

这会输出 `MJCF/env.xml` 的执行器数量、控制范围和传感器列表，适合先确认模型是否正确加载。

### 3. 运行批量实验

```powershell
python run_experiment.py --controller zero --duration 5 --out runs\zero_5s
python run_experiment.py --controller wheel_step --duration 5 --wheel-torque 0.4 --out runs\wheel_step
python run_experiment.py --controller pitch_pd --duration 5 --out runs\pitch_pd
```

每次实验都会输出：

- `log.csv`
- `summary.png`

### 4. 启动交互式仿真

```powershell
python Simulation.py
```

`Simulation.py` 会加载 `MJCF/env.xml`，启动 MuJoCo viewer，并尝试调用 `cpp_control/build-ninja/vmc_bridge.dll`。

## 控制器接口

所有控制器统一返回 6 维力矩向量，顺序固定为：

1. `right_front`
2. `right_rear`
3. `left_front`
4. `left_rear`
5. `right_wheel`
6. `left_wheel`

统一接口形式如下：

```python
torque = controller.compute(obs, t)
```

其中：

- `obs` 是 `environment.LegWheelRobot.get_observation()` 返回的字典
- 返回值应为 `shape=(6,)` 的 `numpy.ndarray`

## 模型说明

- 主场景文件：`MJCF/env.xml`
- 机器人本体：`MJCF/robot.xml`
- 网格、贴图等资源：`MJCF/`
- 当前仿真步长在 `MJCF/env.xml` 中设为 `1 ms`

## C++ 桥接

`cpp_control/` 目录提供了 C++ 版本的 VMC 演示和共享库构建目标：

- `wheel_leg_cpp_control`：原生可视化示例
- `vmc_bridge.dll`：供 `Simulation.py` 加载的动态库

推荐构建方式：

```powershell
cmake -S E:\Mujoco\wheel_leg_mujoco\cpp_control -B E:\Mujoco\wheel_leg_mujoco\cpp_control\build-ninja -G Ninja
cmake --build E:\Mujoco\wheel_leg_mujoco\cpp_control\build-ninja
```

如果 `Simulation.py` 报错找不到 `vmc_bridge.dll`，先完成 C++ 工程构建。

## Git 初始化流程

如果你要按规范把仓库接到 GitHub，推荐流程如下：

```powershell
git branch -M main
git remote add origin git@github.com:Flourish-Violet/Wheel_leg_Mujoco.git
git add README.md
git commit -m "Add Chinese README"
git push -u origin main
```

如果远程已经存在，替换为：

```powershell
git remote set-url origin git@github.com:Flourish-Violet/Wheel_leg_Mujoco.git
```

## 备注

- `runs/` 用于存放实验结果，不建议把大量生成文件直接提交到仓库。
- `__pycache__/`、构建目录和临时文件不建议提交。
- `README_control_lab.md` 是更长的实验说明，`README.md` 是仓库入口文档。
