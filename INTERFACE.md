# MuJoCo 轮腿项目接口说明

## 1. 模型入口

- 主 XML：`MJCF/env.xml`
- 机器人 XML：`MJCF/robot.xml`
- MuJoCo 仿真步长：`0.001 s`
- 车体自由关节：`base_free`，类型为 `free`

## 2. 执行器顺序

`data.ctrl` 的顺序非常重要，后续所有控制器都按这个顺序输出力矩。

| ctrl 下标 | 执行器 | 关节 | 控制范围 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | Right_front_joint_act | jAB | -20..20 | 右前腿关节力矩 |
| 1 | Right_rear_joint_act | jAG | -20..20 | 右后腿关节力矩 |
| 2 | Left_front_joint_act | jIJ | -20..20 | 左前腿关节力矩 |
| 3 | Left_rear_joint_act | jIO | -20..20 | 左后腿关节力矩 |
| 4 | Right_Wheel_act | jwheel_right | -4..4 | 右轮力矩 |
| 5 | Left_Wheel_act | jwheel_left | -4..4 | 左轮力矩，MJCF 中 `gainprm=-1` |

## 3. 传感器名称

| 传感器 | 含义 |
| --- | --- |
| orientation | IMU 四元数，顺序为 wxyz |
| gyro | IMU 角速度 |
| Right_Wheel_pos | 右轮关节位置 |
| Left_Wheel_pos | 左轮关节位置 |
| Right_front_joint_pos | `jAB` 位置 |
| Right_rear_joint_pos | `jAG` 位置 |
| Left_front_joint_pos | `jIJ` 位置 |
| Left_rear_joint_pos | `jIO` 位置 |

## 4. 现有 Python 封装约定

`LegWheelRobot.joint_torque` 使用顺序：

```text
[右前腿, 右后腿, 左前腿, 左后腿]
```

`LegWheelRobot.wheel_torque` 使用顺序：

```text
[右轮, 左轮]
```

`LegWheelRobot.get_observation()` 返回控制和日志记录所需的观测字典。普通控制器优先使用这个观测字典，不建议直接散落读取 `data`，除非需要 MuJoCo 的底层字段。

## 5. 后续算法验证建议

- 先用零力矩和开环轮端力矩确认仿真、传感器、日志链路正常。
- 再加入 pitch PD，观察车体姿态和轮端力矩曲线。
- VMC/LQR 控制器都放到 `controllers.py`，统一实现 `compute(obs, t)`。
- 强化学习环境应该复用同一套观测、动作和限幅逻辑。