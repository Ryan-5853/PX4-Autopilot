# PX4 上的 500 Hz int8 GRU 控制器框架

本文记录 MicoAir H743 V2 上 `nn_control` 占位控制器的模型契约、实时预算、
调试接口和上板验收方法。当前实现用于跑通板载推理并测量负载，不是可飞行模型：
权重由固定种子的伪随机数生成，执行器输出被硬隔离。

## 目标和边界

占位模型是单 batch、单 timestep 的两层 GRU：

```text
21 inputs -> GRU(128) -> GRU(128) -> Linear(4)
```

每次推理保留两层 hidden state，目标频率为 500 Hz，即 2 ms 一步。逻辑模型每步
包含 156,032 次 MAC，持续需求是 78.016 MMAC/s。板级 CPU 是 480 MHz
STM32H743VI，每个 2 ms 周期有 960,000 CPU cycles。

实时验收目标如下：

| 项目 | 目标 |
| --- | ---: |
| 控制频率 | 500 Hz |
| GRU 平均时延 | 不超过 0.8 ms |
| GRU p99 时延 | 不超过 1.0 ms |
| 完整周期 p99 | 不超过 1.2 ms |
| 完整周期最大值 | 不超过 1.5 ms |
| 稳态 deadline miss | 30 分钟内为 0 |

当前代码把推理超过 1 ms 记为 inference overrun，把完整周期超过 1.5 ms 记为
预算余量超限，把超过 2 ms 记为真正的 deadline miss。这些计数用于尽早发现不满足
余量要求的实现。

## 21 维占位输入契约

当前特征顺序固定如下：

| 索引 | 特征 | 坐标/范围 | 占位 inverse scale |
| ---: | --- | --- | ---: |
| 0..3 | attitude quaternion `[w,x,y,z]` | PX4 quaternion | 127 |
| 4..6 | angular velocity | body FRD, rad/s | 32 |
| 7..9 | angular acceleration | body FRD, rad/s² | 4 |
| 10..12 | acceleration | body FRD, m/s² | 8 |
| 13..15 | local velocity | NED, m/s | 16 |
| 16 | RC roll | `[-1,1]` | 127 |
| 17 | RC pitch | `[-1,1]` | 127 |
| 18 | RC yaw | `[-1,1]` | 127 |
| 19 | RC throttle | `[-1,1]` | 127 |
| 20 | battery remaining | `[0,1]` | 127 |

计算方式是 `q = saturate(round(value * inverse_scale), -127, 127)`。为了使第一层
权重行按 4 字节对齐，输入内部补零到 24 维；补齐元素不是模型特征。

这些 scale 只是占位值。训练模型导出时必须一起替换特征顺序、归一化范围、
scale、zero point、bias 和权重，训练端与飞控端应使用同一个模型版本和校验值。

## 四维输出契约

占位物理映射定义为：

```text
output[0] -> Motor 1
output[1] -> Motor 2
output[2] -> Servo 1
output[3] -> Servo 2
```

网络输出范围是 `[-1,1]`。电机通道在发布前转换成 `[0,1]`，舵机保持
`[-1,1]`。当前随机模型编译为永久 dry-run，因此该映射不会真正获得执行器权限，
所有 actuator topic 控制值仍发布为 NaN。

若最终四维输出代表力/力矩而不是物理执行器，此映射必须删除并重新启用
Control Allocator。

## 定点实现

占位实现使用：

- int8 输入、权重和 hidden state；
- PyTorch `reset-after` 形式的 input/recurrent 两套 int32 bias，以及 int32 矩阵累加器；
- Q15 reset/update gate；
- 257 项 Q15 `tanh` 查找表；
- Cortex-M7 上用 `SXTB16 + SMLAD` 完成打包 int8 点积；
- 启动时一次性初始化随机权重；查找表以两端一致的固定 Q15 常量保存；
- 推理路径不做动态内存分配。

随机权重取值为 `[-8,8]`，使用固定 seed `0x4e4e4752`。固定 seed 使每次启动的
权重 checksum、负载和参考输出可复现。权重保存在静态 RAM 中，用于模拟后续将
正式模型复制到 AXI SRAM 后的访问方式。

当前模型存储量约 163 KB。正式模型导出后可以选择：

1. 保持相同 RAM 布局，在启动时由 Flash 复制到 AXI SRAM；
2. 权重留在 Flash，若实测 cache miss 的最大时延仍满足预算则节省 RAM。

hidden state 会在模块启动、手动 reset，以及有效输入中断时清零。

## PX4 调度

`nn_control` 与原生角速度控制器一样运行在最高优先级 `wq:rate_ctrl`，通过
`vehicle_angular_velocity` 的 uORB callback 唤醒。callback 设置了 2 ms
最小间隔，因此数据源超过 500 Hz 时最多运行 500 Hz。

板上测试前必须将陀螺控制数据发布率提高到至少 500 Hz，推荐：

```sh
param set IMU_GYRO_RATEMAX 1000
param save
reboot
```

默认 `IMU_GYRO_RATEMAX=400` 时，只能观察到约 400 Hz 推理，这不是 GRU
计算性能问题。

姿态、加速度、位置、RC 和电池等较慢 topic 采用最新值 sample-and-hold。
RC 未连接时，RC 特征置零，因此 disarmed、无遥控状态仍可以安全跑负载测试。
姿态、角速度和加速度缺失或超时则停止推理并清空 recurrent state。

## 启动和查看负载

编译并烧写：

```sh
make micoair_h743-v2_nncontrol
```

在 NSH 中启动模块：

```sh
nn_control start
```

查看汇总：

```sh
nn_control status
```

输出包含：

- 实测 callback/推理频率；
- dry-run 与执行器隔离状态；
- 模型存储字节数和随机权重 checksum；
- inference 次数和被抑制的输出次数；
- 最近/最大 inference 时延；
- inference 超过 1 ms 的次数；
- 最近/最大完整周期时延；
- 完整周期超过 1.5 ms 的预算超限次数；
- 完整周期超过 2 ms 的 deadline miss 次数；
- 最近四维网络输出；
- PX4 perf counter 的 count、平均值、最小值、最大值和 RMS。

也可以使用全局性能命令查看：

```sh
perf
```

重点关注：

```text
nn_control: cycle
nn_control: inference
nn_control: interval
nn_control: 1.5ms budget overrun
nn_control: 2ms deadline miss
```

清空 hidden state 和统计计数：

```sh
nn_control reset
```

## MAVLink 逐周期一致性验证

`Tools/nn_control/verify_gru_mavlink.py` 在上位机重新生成同一套确定性随机权重，
逐周期向 MCU 发送 21 维观测，并将 MCU 返回的四维结果与本地 int8 GRU
参考实现比较。固件中的验证网络拥有独立 hidden state，不会修改 500 Hz 控制网络
的 recurrent state；随机权重 dry-run 和执行器隔离也不会被验证命令解除。

该链路复用 MAVLink 2 标准消息 `DEBUG_FLOAT_ARRAY`，无需维护自定义 XML：

| 方向 | `name` | `array_id` | `data[]` 内容 |
| --- | --- | ---: | --- |
| 上位机 → MCU | `NNGRU_REQ` | 42420 | 版本、命令、序号、checksum 两个 16-bit word、21 维观测 |
| MCU → 上位机 | `NNGRU_RSP` | 42421 | 版本、状态、序号、纯推理耗时、4 维输出、checksum、权重字节数、MAC 数 |

checksum 拆成两个精确可表示的 16-bit float word 传输，避免把 32-bit 整数直接
放进 float 导致低位丢失。上位机在发送第一条观测之前会执行以下握手：

1. 本地用固定 seed `0x4e4e4752` 初始化权重并计算 FNV-1a checksum；
2. 发送 `INFO`，核对 MCU checksum、163344 字节模型大小和 156032 MAC；
3. 发送 `RESET`，同步清零 MCU 验证实例与本地参考实例的两层 hidden state；
4. 依次发送 `STEP`，每个请求只对应一次推理和一次状态更新；
5. 比较四维结果，并汇总 MCU 推理耗时、MAVLink RTT 和最大绝对误差。

如果 checksum 不同、MCU 拒绝请求、响应超时或某一周期输出不一致，脚本立即停止。
超时后不自动重发 `STEP`，因为盲目重发会让其中一端的 GRU state 多前进一步；
重新运行脚本会再次执行双方 `RESET`。

先烧写并启动固件：

```sh
make micoair_h743-v2_nncontrol
nn_control start
```

USB CDC/串口连接示例：

```sh
Tools/nn_control/verify_gru_mavlink.py /dev/ttyACM0 \
  --baud 115200 --cycles 1000 --print-every 50
```

UDP 连接示例：

```sh
Tools/nn_control/verify_gru_mavlink.py udp:127.0.0.1:14550 \
  --cycles 1000 --print-every 50
```

脚本依赖 `numpy` 和 `pymavlink`，可按 PX4 的 Python 依赖清单安装：

```sh
python3 -m pip install -r Tools/setup/requirements.txt
```

默认申请 `DEBUG_FLOAT_ARRAY` 以 20 Hz 输出并串行执行请求/响应，该速率也适合
57600 baud 的遥测串口；带宽充足的 USB/UDP 链路可用 `--response-rate` 提高。它
不是控制环频率测试，
而是有状态网络的数值验证。验证推理本身仍在 `wq:rate_ctrl` 内执行，单次额外占用
约一趟 GRU 的时间。需要测量纯 500 Hz 控制负载时不要同时运行此脚本。

验证期间可以在 NSH 查看独立统计：

```sh
nn_control status
```

其中 `verification requests/failures` 与 `verification inference` 分别显示请求计数、
失败计数及验证实例的最近/最大/平均推理耗时。

## 上板验收顺序

1. 不接桨叶，保持 disarmed，确认 status 显示 `RANDOM-WEIGHT DRY-RUN`。
2. 确认 `IMU_GYRO_RATEMAX=1000`，实测频率接近 500 Hz。
3. 静置运行 10 分钟，观察 inference 和完整周期的平均/最大值。
4. 同时打开 MAVLink、正常日志和 SD 写入，继续运行 30 分钟。
5. deadline miss 必须为 0，完整周期最大值应低于 1.5 ms。
6. 使用 `top`、`work_queue status` 和 `perf` 检查 `wq:rate_ctrl` 是否有积压。
7. 替换正式权重后，先做板端与 Python reference 的逐步数值对比。
8. 正式模型仍应先保持 dry-run；只有接口、量化、超时复位和输出安全测试全部
   通过后，才能单独评审执行器使能。

## 当前已知限制

- 随机模型只能用于测量负载，输出没有控制意义。
- 占位量化 scale 没有经过校准。
- 当前 `tanh/sigmoid` LUT 没有做线性插值，正式模型精度需要重新验证。
- status 中的平均频率基于 PX4 interval perf counter，长时间运行后是累计平均值。
- 500 Hz 推理不代表舵机物理刷新率也能达到 500 Hz；PWM/舵机协议必须单独确认。
- 真实的 p99 需要板上采样或直方图统计，不能仅由主机编译结果推断。
