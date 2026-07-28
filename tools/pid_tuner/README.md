# PID 在线调参验证工具

该目录是独立的 PC 侧最小协议工具，不依赖 Jetson 比赛服务。当前仅提供协议编解码和上板验证脚本，后续 CLI/GUI 将复用 `pid_tuner.protocol`。

## 环境

在 Windows 11 的 `low_numpy` 环境中安装：

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python -m unittest discover -s tests
```

USART1 参数为 `115200 8N1`。调参固件已关闭 USART1 上的文本 `printf`，该串口仅可使用二进制协议客户端连接。

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

遥测仅在远程 `GOTO_POSE` 任务处于 `RUNNING` 时发送，默认监听窗口为 30 秒。若任务提前到达、超时或失联，脚本会如实报告实际收到的帧数、CRC 错误、序号缺失和固件侧遥测覆盖计数。

现场测试前须确保机械急停、断电能力和人员监护可用。GUI STOP 及串口 STOP 都不是物理急停。
