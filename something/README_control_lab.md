# 轮腿 MuJoCo 控制算法实验说明

这个项目已经整理成“先验证控制算法，再过渡强化学习”的实验结构。你可以先在 MuJoCo 里验证 VMC、PID、LQR、状态反馈等控制算法，等控制基线稳定后，再封装成 Gymnasium 环境做强化学习。

## 1. Python 环境

建议使用 Python 3.10 或 3.11。当前系统默认 `python` 是 3.14，很多 MuJoCo/RL 包可能还没有稳定支持，所以不要直接用系统默认 Python 做实验。

如果机器上已有 Python 3.11：

```powershell
cd E:\Mujoco\wheel_leg_mujoco
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

如果 `py -3.11` 找不到 Python 3.11，需要先安装 Python 3.11，再执行上面的命令。

## 2. 运行方式

可视化模式，保留原项目的观察方式：

```powershell
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

## 5. 当前状态

- `MJCF/env.xml` 已通过 MuJoCo 官方 `compile.exe` 编译验证，模型文件本身可加载。
- 当前机器尚未成功安装 Python 3.11；直连 python.org 下载官方安装器时下载不完整。
- 你可以手动下载 `python-3.11.9-amd64.exe` 后放到 `D:\APP\Mujoco`，再继续安装。