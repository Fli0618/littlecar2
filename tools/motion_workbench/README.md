# 底盘运动调试工作台

该工具在一个 PySide6 窗口中组合现有 PID 调参、地图编辑和连续路径调试能力。它不直接访问串口；所有通信都经由 `pid_tuner` 的单一 `SessionController`。

开发环境依次安装三个本地包：

```powershell
pip install -e tools/pid_tuner
pip install -e tools/map_planner
pip install -e tools/motion_workbench
```

启动命令：`motion-workbench`；也可在本目录执行 `python launch_gui.py`。

连续路径使用协议 V2 的扩展命令，要求 STM32 已刷入对应固件。单条路径最多 256 个 `x/y/yaw` 点；工作台在上传前拒绝超限路径。
