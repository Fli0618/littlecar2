# LittleCar2 Motion Studio 0806 (底盘运动 Studio 调试工作台)

本工具为重构后的轻量级、高性能底盘运动调试工作台，放在 `tools/motion_studio0806/` 目录下，彻底解决旧版 `motion_workbench` 功能杂乱与主线程卡顿问题。

## 核心架构与 3 大 Tab 功能

1. **📌 单点 / 定点调试 (Single Point Tab)**
   - 支持 **Holonomic Profile (全向位置控制器)** 与 **Classic Point-to-Point (经典单点 PID)** 两种模式下拉自由切换对比。
   - 提供 Holonomic 12 项运行时热加载参数（加速度上限、Kp/Kv、Scale）。
   - 实时绘制单点收敛遥测图表。

2. **🛣️ 路径追踪与性能测试 (Path Follow Tab)**
   - 内置 **路径模版库（元素存档）**：包含标准 S 弯、90度直角弯、45度蟹行斜平移，支持画好的路径存为新模版。
   - 运动参数实时调节（前瞻距离 Lookahead、横向纠偏 Kp、巡航速度上限）。
   - 双遥测图表：X-Y 轨迹对比图 + 横向偏离误差 (CTE Error) 与速度曲线。

3. **🗺️ 比赛地图与路线规划 (Map Planner Tab)**
   - 2D 高性能地图画布，完全解耦主线程，流畅拖拽加点挪点。
   - 障碍物与路径节点精细表格（支持敲数字精调）。
   - 一键动作：【发送至 Path 调试页测试】与【直接下发小车运行】。

## 独立安装与启动

```powershell
conda run -n low_numpy pip install -e tools/motion_studio0806
conda run -n low_numpy python tools/motion_studio0806/launch_gui.py
```
