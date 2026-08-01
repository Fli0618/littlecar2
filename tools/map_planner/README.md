# 比赛地图路径规划工具

独立的 PySide6 赛前规划与仿真工具，不连接串口，也不会向小车下发指令。

地图按官方 `2400 mm × 2400 mm` 图纸绘制。新建方案后选择启停区或自定义起点，再拖动蓝色流程箭头完成朝向标定。路线由顺序直线 GOTO Pose 节点组成：绿色节点为当前可编辑节点，历史节点为橙色，右键可切换为当前节点。

- `选择`：框选，Ctrl 增减选择，`Ctrl+A` 全选节点，`Ctrl+Z` 撤销。
- `添加节点`：点击添加；按住 Shift 吸附至相对前一节点的水平、垂直或 45 度方向。
- 中键拖拽或空格加左键平移地图；滚轮只缩放地图。
- 每个节点默认只进行位置 GOTO 并到点停止；拖动节点的圆形旋转柄会启用航向约束。

在项目根目录安装并启动：

```powershell
conda run -n low_numpy pip install -e tools/map_planner
conda run -n low_numpy littlecar-map-planner
```

离屏测试：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
conda run -n low_numpy python -m pytest tools/map_planner/tests -q
```
