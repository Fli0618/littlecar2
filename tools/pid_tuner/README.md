# PID 在线调参验证工具

该目录是独立的 PC 侧最小协议工具，不依赖 Jetson 比赛服务。当前仅提供协议编解码和上板验证脚本，后续 CLI/GUI 将复用 `pid_tuner.protocol`。

## 环境

在 Windows 11 的 `low_numpy` 环境中安装：

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python -m unittest discover -s tests
```

USART1 参数为 `115200 8N1`。调参固件已关闭 USART1 上的文本 `printf`，该串口仅可使用二进制协议客户端连接。

## 查询 COM 口

连接 USB 转串口模块后执行以下命令，输出中例如 `COM5` 的端口名可直接传给验证脚本：

```powershell
conda run -n low_numpy python scripts/list_ports.py
```

## 正式 CLI

安装或更新本工具后，可使用 `pid-tuner` 子命令：

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy pid-tuner get-pid --port COM4
conda run -n low_numpy pid-tuner set-pid --port COM4 --kp-pos 1 --ki-pos 0.03 --kd-pos 0.1 --kp-yaw 2 --ki-yaw 0.05 --kd-yaw 0.08 --apply
```

`set-pid` 和 `restore-pid` 未给出 `--apply` 时只进行参数预览。运动命令要求完整的目标、速度和超时参数，并在进程存活期间自动心跳：

```powershell
conda run -n low_numpy pid-tuner goto --port COM4 --x 200 --y 0 --yaw 0 --vmax 50 --wmax 30 --timeout 5000 --csv logs/run.csv
```

`profile save|list|show|export-c` 管理本地 PID JSON 方案；方案默认保存在 `profiles/`，遥测 CSV 默认建议保存至 `logs/`。两类运行数据都被 Git 忽略。

## 实时 GUI

安装依赖后运行：

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python launch_gui.py
```

启动后使用界面左上方的串口下拉框选择 COM 口并点击“连接”，不需要在启动命令中填写端口。GUI 使用现有世界位姿遥测：X/Y 是经坐标修正后的 OPS 实际位置，yaw 在 WIT 有效时优先使用 WIT，否则使用 OPS。固件启动后持续发送遥测，因此仅连接串口、不发送 GOTO 时图表也会显示实时状态。默认显示最近 30 秒，窗口可调 5-120 秒，缓存只保留最近 120 秒。

运动控制必须在现场具备机械急停与人员监护时使用。断开、通信错误和窗口关闭会尝试发送 STOP；这不能替代物理急停。

## 只读验证

默认只验证 `GET_PID` 和坏 CRC 恢复，不会改写 PID，也不会驱动车辆：

```powershell
conda run -n low_numpy python verify_board.py --port COM5
```

## 有风险的验收操作

`--write-pid` 会写入当前 PID 后执行 `RESTORE_PID`，最终参数将回到固件默认值；仅在允许覆盖现场参数时使用。

```powershell
conda run -n low_numpy python verify_board.py --port COM5 --write-pid
```

运动测试必须显式给出绝对目标坐标。脚本固定使用 `50 mm/s`、`30 deg/s`、`5 s` 的低速限制，每 500 ms 发送心跳，并在退出运动测试前发送 `STOP`。

```powershell
conda run -n low_numpy python verify_board.py --port COM5 --exercise-motion --x 200 --y 0 --yaw 0 --csv logs/run.csv
```

遥测在固件启动后以 40 ms 周期持续发送，默认监听窗口为 30 秒。若任务提前到达、超时或失联，脚本会如实报告实际收到的帧数、CRC 错误、序号缺失和固件侧遥测覆盖计数。GOTO 线速度不得超过 1500 mm/s。

现场测试前须确保机械急停、断电能力和人员监护可用。GUI STOP 及串口 STOP 都不是物理急停。
