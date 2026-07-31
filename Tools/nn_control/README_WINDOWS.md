# Windows 独立验证工具包

这个目录可脱离 PX4 源码单独复制到 Windows。它负责通过 MAVLink 向飞控发送
21 维观测，接收 MCU 推理耗时和四维结果，并与本地 bit-equivalent int8 GRU
逐周期比较。

## 目录内容

```text
verify_gru_mavlink.py     主验证程序和本地 GRU 参考实现
requirements-windows.txt 固定版本的 Python 依赖
setup_windows.bat         一键创建虚拟环境、安装依赖并运行自检
check_environment.bat     显示版本、串口并执行本地模型自检
run_verification.bat      串口/UDP 验证启动器
firmware/                 配套 MicoAir H743 V2 固件和构建信息
README_WINDOWS.md         本说明
```

`.venv` 不应从 Linux 复制，也不应在不同 Windows 电脑之间复制。每台 Windows
电脑都运行一次 `setup_windows.bat`，由它下载该平台对应的 wheel。

## 系统要求

- 64-bit Windows 10 或 Windows 11；
- 64-bit CPython 3.12；
- 安装依赖时可访问 PyPI；
- USB CDC、USB 转串口或可达的 UDP MAVLink 链路；
- 飞控已烧写带 `NNGRU_REQ/NNGRU_RSP` 适配的固件；
- 飞控 NSH 中 `nn_control` 已启动，并且 IMU 正常发布角速度。

建议从 Python 官方网站安装 Python 3.12。安装时启用 Python Launcher。可在
命令提示符检查：

```bat
py -0p
py -3.12 --version
```

## 第一次配置

将整个目录复制到 Windows 后，双击：

```text
setup_windows.bat
```

它会：

1. 检查 Python 3.12；
2. 在当前目录创建 `.venv`；
3. 安装固定版本的 NumPy、pymavlink 和 pyserial；
4. 运行 100 个周期的本地 recurrent 自检；
5. 列出系统发现的串口。

成功的模型自检必须包含：

```text
Local weights: 163344 bytes, checksum=0xb8ea4fee
Local self-test PASSED
```

如果需要重新配置，删除 `.venv` 目录后再次运行 `setup_windows.bat`。不要修改
`verify_gru_mavlink.py` 中的 seed、权重生成顺序、量化 scale 或固定 LUT。

## 飞控准备

工具包的 `firmware` 目录包含本次编译的：

```text
micoair_h743-v2_nncontrol.px4
```

可使用 QGroundControl 的 Firmware → Advanced settings → Custom firmware file
选择该文件。它仅适用于 MicoAir H743 V2，不能烧写到其他硬件。

烧写后在 NSH 执行：

```sh
nn_control start
nn_control status
```

状态应包含：

```text
21-128-128-4 int8 GRU
weights: 163344 bytes
checksum: 0xb8ea4fee
```

验证处理依靠 `vehicle_angular_velocity` 回调运行。如果 IMU 没有数据，即使
MAVLink heartbeat 正常，验证请求也不会被处理。

确保正在使用的 MAVLink 实例允许 `DEBUG_FLOAT_ARRAY`。脚本连接后会发送
`MAV_CMD_SET_MESSAGE_INTERVAL`，将该消息请求为默认 20 Hz。

## 串口运行

先查看 Windows 设备管理器中的端口号，例如 `COM5`。可以双击
`run_verification.bat`，按提示输入端口；也可以从命令提示符运行：

```bat
run_verification.bat COM5
```

可选的第二、第三参数分别是 baud 和周期数：

```bat
run_verification.bat COM5 115200 1000
run_verification.bat COM12 921600 5000
```

USB CDC 虚拟串口通常忽略 baud 参数；USB-UART/TELEM 连接必须与 PX4 端口配置
保持一致。57600 baud 时建议保持 20 Hz，不要盲目提高 `--response-rate`。

也可以绕过启动器直接运行：

```bat
".venv\Scripts\python.exe" verify_gru_mavlink.py COM5 ^
  --baud 115200 ^
  --cycles 1000 ^
  --response-rate 20 ^
  --print-every 50
```

## UDP 运行

如果 MAVLink 已由路由器或地面站转发到本机 UDP 14550：

```bat
run_verification.bat udp:127.0.0.1:14550 115200 1000
```

UDP 模式下 baud 参数不会影响链路。

## 环境诊断

双击：

```text
check_environment.bat
```

它会输出 Python/依赖版本、检测到的 COM 端口，以及本地权重和 recurrent
执行自检。也可以单独列出串口：

```bat
".venv\Scripts\python.exe" -m serial.tools.list_ports -v
```

## 通过条件

脚本先执行 `INFO` 和 `RESET`，随后按序执行 `STEP`。最终应显示：

```text
Verification PASSED
MCU inference [us]: ...
MAVLink RTT [ms]: ...
Output error: ... max=... tolerance=0.000020000
```

以下任一情况都会使验证失败：

- MCU checksum 不是 `0xb8ea4fee`；
- MCU 模型大小不是 163344 bytes；
- MCU MAC 数不是 156032；
- 响应状态不是 `OK`；
- 请求超时；
- 任一周期四维结果的绝对误差超过容差。

发生 `STEP` 超时时脚本不会自动重发。MCU 可能已经更新 hidden state，只是响应
丢失；自动重发会令两端 recurrent state 错位。排除链路问题后重新运行脚本，
它会重新发送 `RESET`。

## 常见问题

### 找不到 heartbeat

- 检查 COM 端口是否正确；
- 关闭 QGroundControl、Mission Planner、串口终端等可能独占同一 COM 口的软件；
- 检查 baud 是否与 PX4 端口配置一致；
- 检查 MAVLink 是否运行在该串口；
- 如果通过路由器转发，确认目标 UDP 地址和端口。

### 有 heartbeat，但等待 `NNGRU_RSP` 超时

- 确认执行了 `nn_control start`；
- 确认 `nn_control status` 的运行频率不为零；
- 确认 IMU 正常发布 `vehicle_angular_velocity`；
- 确认烧写的是本工具包配套固件；
- 在 NSH 中查看 `nn_control status` 的 verification request 计数是否增长；
- 确认 MAVLink 实例没有禁用 `DEBUG_FLOAT_ARRAY`。

### 安装 pymavlink 失败

- 确认使用 64-bit CPython 3.12，而不是 Microsoft Store 的受限别名；
- 先运行 `py -3.12 -m pip install --upgrade pip`；
- 确认网络/代理允许访问 PyPI；
- 不要删除 `--only-binary=:all:` 后尝试本地编译，正常配置应直接下载 Windows wheel。

### COM10 以上端口

pymavlink/pyserial 接受 `COM12` 这样的端口名，通常不需要手动写成
`\\.\COM12`。

## 安全说明

当前固件仍是 RANDOM-WEIGHT DRY-RUN，验证接口使用独立 hidden state，不会解除
执行器隔离。测试时仍应拆除桨叶并保持 disarmed。MAVLink 验证会在收到 `STEP`
的控制周期额外执行一次 GRU，因此不要用验证期间的完整周期统计代替纯 500 Hz
负载测试。
