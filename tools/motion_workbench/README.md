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

地图的“显示交换 X/Y”和“显示反转 X/Y”只转换实测车辆、实测轨迹与地图误差，不修改路径点、原始遥测或下发命令，且每次启动默认关闭。变换固定先交换、再反转，车辆航向随方向向量同步变换。单步执行始终运行当前选中动作，可重复点击；连贯执行从当前选中动作开始。

“路径”页可在线读取、整组应用或恢复 14 项连续路径参数，包括横向/航向 PD、速度与加速度规划、动态前视。新参数由 STM32 在 20 ms 控制周期边界切换，活动路径的进度不会被清空。
