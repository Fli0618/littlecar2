# 比赛地图路径规划工具

独立的 Windows PC 工具，用于 LittleCar2 决赛场地上的路径编辑、运动仿真和方案保存。工具不会连接串口或控制车辆。

在 `low_numpy` 环境安装并启动：

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy littlecar-map-planner
conda run -n low_numpy python -m unittest discover -s tests
```

场地为 2400 mm x 2400 mm。起始车辆中心是世界坐标原点，车辆前方为 `+Y`、右侧为 `+X`、逆时针航向为正。方案 JSON 默认保存到本目录的 `plans/`，该目录不提交到 Git。
