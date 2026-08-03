# 轮腿 MuJoCo 控制算法实验说明

这个项目已经整理成“先验证控制算法，再过渡强化学习”的实验结构。你的主线是：先用 MuJoCo 验证 VMC、PID、LQR、状态反馈等控制算法，等控制基线稳定后，再封装成 Gymnasium 环境做强化学习。

## 1. Conda 环境

推荐使用 Conda 管理 Python 版本和隔离环境。当前已把 Conda 环境、包缓存和 pip 缓存统一设置到 `E:\CondaData`。不要直接用系统默认 `python`，当前系统默认版本是 3.14，很多 MuJoCo/RL 包可能还没有稳定支持。

安装 Anaconda、Miniconda 或 Miniforge 后，打开 Anaconda Prompt 或 PowerShell，执行：

```powershell
conda create -n mujoco311 python=3.11
conda activate mujoco311
cd E:\Mujoco\wheel_leg_mujoco
python -m pip install -r requirements.txt
```

环境验证：

```powershell
python --version
python -c "import mujoco; print(mujoco.__version__)"
python run_experiment.py --inspect
```

说明：Conda 主要负责创建 `mujoco311` 和安装 Python 3.11；新环境会默认创建到 `E:\CondaData\envs\mujoco311`，包缓存会进入 `E:\CondaData\pkgs`，pip 缓存会进入 `E:\CondaData\pip-cache`。`mujoco`、`gymnasium`、`stable-baselines3` 这类机器人/RL 包建议用 pip 安装到该 Conda 环境里。

## 2. 运行方式

可视化模式，保留原项目的观察方式：

```powershell
conda activate mujoco311
cd E:\Mujoco\wheel_leg_mujoco
python Simulation.py
```

无窗口零力矩基线实验，适合重复记录数据：

```powershell
python run_experiment.py --controller zero --duration 5 --out runs\zero_5s
```

开环轮端力矩实验：

```powershell
python run_experiment.py --controller wheel_step --duration 5 --wheel-torque 0.4 --out runs\wheel_step
```

检查模型接口：

```powershell
python run_experiment.py --inspect
```

## 3. 控制器接口

所有控制器统一实现：

```python
torque = controller.compute(obs, t)
```

返回值是 6 路力矩，顺序固定为：

1. 右前腿关节，`jAB`，执行器 `Right_front_joint_act`，范围 `[-20, 20]`
2. 右后腿关节，`jAG`，执行器 `Right_rear_joint_act`，范围 `[-20, 20]`
3. 左前腿关节，`jIJ`，执行器 `Left_front_joint_act`，范围 `[-20, 20]`
4. 左后腿关节，`jIO`，执行器 `Left_rear_joint_act`，范围 `[-20, 20]`
5. 右轮，`jwheel_right`，执行器 `Right_Wheel_act`，范围 `[-4, 4]`
6. 左轮，`jwheel_left`，执行器 `Left_Wheel_act`，范围 `[-4, 4]`，注意 MJCF 中 `gainprm=-1`

## 4. 推荐学习顺序

1. 先加载 `MJCF/env.xml`，确认模型维度、关节、执行器和传感器名称。
2. 跑 `zero` 和 `wheel_step` 两个无窗口基线，查看 CSV 和 PNG 曲线。
3. 在 `controllers.py` 里继续加入 PID、VMC、LQR 控制器。
4. 每个实验保持同样的初始条件、仿真时长、记录字段，方便横向比较。
5. 控制基线稳定后，再把这个项目封装成 Gymnasium 环境做强化学习。

## 5. 后续强化学习环境

控制算法阶段跑通后，再在同一个 Conda 环境里安装 RL 包：

```powershell
conda activate mujoco311
python -m pip install gymnasium[mujoco] stable-baselines3
```

建议先不要急着训练 PPO/SAC。先把观测、动作、奖励、终止条件设计清楚，再封装 Gymnasium 环境。

## 6. 当前状态

- `MJCF/env.xml` 已通过 MuJoCo 官方 `compile.exe` 编译验证，模型文件本身可加载。
- 项目已提供 `run_experiment.py`，可在安装好 Conda 环境后做无窗口重复实验。
- 默认实验环境名：`mujoco311`。