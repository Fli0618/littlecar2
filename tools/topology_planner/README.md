# LittleCar2 拓扑路径规划

这是一个独立的比赛场地拓扑路径规划工具，不连接小车、不下发运动指令、不执行视觉检测，也不进行连续坐标轨迹规划。

## 拓扑

画布固定显示 `NW/N/NE`、`W/C/E`、`SW/S/SE` 九个导航节点，以及 `RAW` 原料区、`QR` 二维码区、`ROUGH` 粗加工区、`BUFFER` 暂存区、`START1/START2` 启停区六个任务叶子节点。导航节点之间有 12 条边，任务节点各连接一个导航节点，共 18 条无向边。

点击白色道路可禁用道路（显示为红色），再次点击可恢复。任务节点不能作为路径中间节点。

## 成本

路径长度按逐边欧式距离累加，默认导航边长度为 `1.0`、任务边长度为 `0.5`。方向变化按 90 度转向单位统计，并在转向处统计停车次数：

```text
总成本 = A × 路径长度 + B × 90 度转向单位 + C × 停车次数
```

默认 `A=1.00`、`B=0.75`、`C=1.00`，界面可以实时调整。工具使用 DFS 枚举简单路径，按总成本及稳定次级键排序，最多显示四条候选路径。

## 安装和启动

在仓库根目录执行：

```powershell
conda run -n low_numpy pip install -e tools/topology_planner
conda run -n low_numpy littlecar-topology-planner
```

也可以双击或直接运行 `launch_gui.py`：

```powershell
conda run -n low_numpy python tools/topology_planner/launch_gui.py
```

## 测试

```powershell
$env:QT_QPA_PLATFORM='offscreen'
conda run -n low_numpy python -m pytest tools/topology_planner/tests -q
```
