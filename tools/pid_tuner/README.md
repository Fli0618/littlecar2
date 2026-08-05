# PID 在线调参验证工具

该目录是独立的 PC 侧协议、串口客户端、会话和 GUI 工具，不依赖 Jetson 比赛服务。协议 V3 保持现有命令号、载荷顺序、CRC 和 revision 语义。

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
conda run -n low_numpy pid-tuner goto --port COM4 --x 200 --y 0 --yaw 0 --vmax 600 --wmax 120 --timeout 15000 --csv logs/run.csv
```

`profile save|list|show|export-c` 管理本地 PID JSON 方案；方案默认保存在 `profiles/`，遥测 CSV 默认建议保存至 `logs/`。两类运行数据都被 Git 忽略。

## 实时 GUI

GUI 的 GOTO 区域提供“航向误差大时先对准”开关。它通过 USART1 临时修改板端策略，仅在位置和航向同时启用的 GOTO 中生效；板端复位后恢复 `advance_motion.h` 的编译期默认值。运动进行时开关不可修改。

安装依赖后运行：

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python launch_gui.py
```

启动后使用界面左上方的串口下拉框选择 COM 口并点击“连接”，不需要在启动命令中填写端口。GUI 持续显示 OPS X/Y、WIT 相对航向和 OPS 相对 Z 航向；两套航向均与同一 GOTO 目标角对比。“航向控制模式”可选 `WIT yaw`、`OPS yaw` 或“不使用航向”：前两项通过现有协议切换 STM32 航向源，后一项使后续主 GOTO 只携带 X/Y 位置约束，不输出 yaw 控制。切换 WIT/OPS 会重置航向 PID 历史，运动中不能切换模式。GOTO 默认目标为 `(0, 0, 0)`，调试最高限速为 `1200 mm/s`、`120 deg/s`，默认 vmax 为 `600 mm/s`，默认超时为 `15000 ms`。除主 GOTO 外，界面还保留显式的仅位置和仅角度 GOTO；选择“不使用航向”时 yaw 输入、角速度输入和仅角度 GOTO 自动禁用。重置零点仅在空闲时成功，并会将当前 OPS X/Y、WIT 航向与 OPS 航向同时归零。每次“应用 PID”获得 STM32 确认后，启动 GUI 的终端会输出修订号与六个 PID 参数。误差图显示零误差虚线、最新数值，并标注 WIT/OPS 是当前控制源、对照源还是未参与控制。默认显示最近 30 秒，窗口可调 5-120 秒，缓存只保留最近 120 秒。

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

遥测在固件启动后以 40 ms 周期持续发送，默认监听窗口为 30 秒。若任务提前到达、超时或失联，脚本会如实报告实际收到的帧数、CRC 错误、序号缺失和固件侧遥测覆盖计数。位置环隔离测试可在 `goto` 后添加 `--no-yaw`；当前 PID 调试使用 `1200 mm/s`、`120 deg/s` 作为最高限速。

现场测试前须确保机械急停、断电能力和人员监护可用。GUI STOP 及串口 STOP 都不是物理急停。

## 全向参数与统一导出

GUI 的“全向位置”页对应 `AdvanceHolonomic_Config_t` 的 12 项字段，使用 `GET/SET/RESTORE_HOLONOMIC_CONFIG`（`0x29` 至 `0x2B`）读写。`SET` 和 `RESTORE` 收到 ACK 后会轮询 active revision；只有板端在周期边界报告目标 revision，界面才显示“已生效”。

工作台导出函数 `export_motion_config_header` 接收 PID、路径、GOTO 策略和全向四类已确认状态，一次生成 `advance_motion_config.h`。导出内容包含四类 revision、6 项 PID、20 项路径、1 项 GOTO 策略和 12 项全向默认宏；不再生成或读取 `advance_holonomic_position_defaults.h`。

旧固件对 `GET_HOLONOMIC_CONFIG` 返回 `BAD_COMMAND` 时，Session 保留串口连接并仅禁用全向页面，PID、路径和地图功能继续可用。
