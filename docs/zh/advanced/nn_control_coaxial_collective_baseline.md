# 稳定单体控制基线

本文标记 2026-10-30 的 `nn_control` 与双电机总距/差模、三舵机姿态
LQI 为 **airframe-1 稳定单体控制基线**。

这里的“单体控制”是指：单个已标定机架在手动姿态与总距模式下的滚转、俯仰、
偏航角速度和竖直加速度控制，不包含水平位置、水平速度或航向角保持。

## 基线标识

```text
airframe:        airframe-1
NN_LQI_MODEL:    2
NN_AIRFRAME_ID:  1
model checksum:  0x78ae1111
firmware SHA256: 0453336d9c3fd7694581955886145f2d54483635cb97ac60b6e215da1d366ff0
firmware size:   1061994 bytes
```

基线提交对应 MicoAir H743 V2 `micoair_h743-v2_nncontrol` 固件。

## 控制契约

物理输出保持：

```text
[upper_motor, lower_motor, servo_1, servo_2, servo_3]
```

- 飞手总距映射为竖直加速度指令；
- z 加速度 PI 控制双电机共模推力；
- 电机差模只用于 yaw；
- 三个舵机只分配滚转和俯仰力矩；
- 手动 roll/pitch 参考不再进行一阶杆量滤波；
- 手动 roll/pitch 最大目标角为 20°；
- yaw 为角速度控制，目标上限为 1.2 rad/s。

该架构用于后续位置/速度外环接入前的单体基线，不应与多机架或完整自主飞行
认证混淆。

## 日志配置

为避免 SD 写入造成 ULog dropout，稳定基线使用压缩动态日志：

| 主题 | 频率 |
| --- | ---: |
| attitude, angular velocity | 200 Hz |
| actuator motors, servos | 200 Hz |
| ESC status | 50 Hz |
| vehicle acceleration | 100 Hz |
| dual raw accelerometers | 50 Hz each |
| manual control | 50 Hz |
| estimator diagnostics | 5 Hz |

200 Hz 足以分析当前 3–8 Hz 刚体动态，同时显著降低日志写入负载。

## 验证状态

- 固定非线性 SimEnv 场景全部通过；
- 姿态回正与跟踪误差 p95 小于 0.2°；
- 总距 z 加速度 RMSE 约 0.069 m/s²；
- 舵机饱和比例均值/最大值约 0.04% / 0.48%；
- PX4 host LQI 黄金向量通过；
- `micoair_h743-v2_nncontrol` 完整构建通过；
- 真实飞行日志无 dropout，新的 200/100/50 Hz 日志配置有效。

## 已知边界

- 飞手滚转杆是姿态角指令，不是水平速度或位置指令；
- 满杆只能请求 20°倾角，不能继续增大横向修正；
- 总距进入 `collective <= 0.02` 后会同时关闭电机和舵机；
- 当前没有水平位置/速度外环，不能主动消除既有平移速度；
- 当前 yaw 仅做角速度阻尼，不保持航向角；
- 本次硬件日志中没有发现可用磁力计，EKF yaw 未完成磁校准；
- 不能把本基线当作多机架通用模型或完整位置控制飞行许可。

接入外部位置/速度环前，必须提供有效的水平位置或速度源，并单独验证航向参考。
