# LittleCar2 拓扑路径规划器

这是一个独立的离线拓扑规划与任务链仿真工具。它不连接小车、不下发运动指令、不解析真实任务码，也不生成 STM32 运动函数。

## 两种工作模式

### 单段路径

单段模式用于在固定 3x3 拓扑中选择任意合法起点和终点，并显示最多四条候选路径。左键点击道路可以切换禁用状态，再次点击可恢复。A/B/C 权重分别表示路径长度、90 度转向单位和停车次数，成本公式为：

```text
总成本 = A × 路径长度 + B × 转向单位 + C × 停车次数
```

导航边长度为 `1.0`，任务叶子边长度为 `0.5`。这些是拓扑长度单位，不是毫米距离。

### 完整任务链

完整任务链模式只允许选择 `START1` 或 `START2`，任务顺序固定为：

```text
STARTx → QR → RAW → ROUGH → BUFFER → RAW → ROUGH → BUFFER → STARTx
```

工具会对每两个相邻任务点复用单段 `find_best_paths()`，在当前禁用道路和权重下选择最低成本路径。任务点会重复访问，路径拼接只去掉相邻段连接处的一个重复节点，不会按节点 ID 全局去重。

每个任务点的离线停留时间默认为 `0.8` 秒；起点和最终返回点不额外停留。任务动作仅用于界面显示，不代表真实机械动作耗时。

## 仿真控制

生成完整任务链后，可使用播放、暂停/继续、停止复位和 `0.5x/1x/2x/4x` 倍速控制。进度条按累计拓扑路径长度计算：

```text
进度 = 已行驶拓扑长度 / 完整任务链拓扑长度
```

小球只是在拓扑边上做线性插值，用于离线演示。仿真器是纯 Python 状态机，不依赖 Qt；GUI 使用 30ms 定时器和单调时钟计算真实 `dt`，不会每帧重建场景、路线或任务列表。

任务链完成后状态为 `FINISHED`，小球回到原始启停区，进度为 100%；再次播放会从头重播。暂停期间调用 `tick()` 不会推进位置，负 `dt` 和非正速度倍率会被拒绝。

## 不可达任务链

如果某一任务段在当前道路配置下不可达，生成立即失败并报告失败段编号、起点、终点和原因。此时任务计划、小球、任务路线和进度会被清空，播放按钮保持禁用；单段路径模式仍可继续使用。修改起始区、权重或禁用道路都会使旧任务计划失效，必须重新生成。

## 安装与启动

在仓库根目录执行：

```powershell
conda run -n low_numpy pip install -e tools/topology_planner
conda run -n low_numpy littlecar-topology-planner
```

也可以直接运行：

```powershell
conda run -n low_numpy python tools/topology_planner/launch_gui.py
```

## 测试

PowerShell 下使用 Qt 离屏模式运行：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
conda run -n low_numpy python -m pytest tools/topology_planner/tests -q
```

额外检查：

```powershell
conda run -n low_numpy python -m py_compile `
  tools/topology_planner/topology_planner/planner.py `
  tools/topology_planner/topology_planner/mission.py `
  tools/topology_planner/topology_planner/simulation.py `
  tools/topology_planner/topology_planner/gui.py
git diff --check
```
