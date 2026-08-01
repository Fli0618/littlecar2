# 比赛地图路径规划工具

独立的 PySide6 赛前规划与仿真工具，不连接串口，也不会向小车下发指令。

地图按官方 `2400 mm × 2400 mm` 图纸绘制。四个黄色区的定位坐标由“场地中心对称、十字通道居中”关系推导为 `(550,550)`、`(1400,550)`、`(550,1400)`、`(1400,1400)`。新建方案必须先选择启停区或自定义起点，再拖动蓝色流程箭头完成朝向标定。

- `选择`：框选，Ctrl 增减选择，`Ctrl+A` 全选节点，`Ctrl+Z` 撤销。
- `添加节点`：点击添加；按住 Shift 吸附至相对前一节点的水平、垂直或 45 度方向。
- 中键拖拽或空格加左键平移地图；滚轮只缩放地图。
- 每个节点默认只进行位置 GOTO 并到点停止；拖动节点的圆形旋转柄会启用航向约束。
- 框选或 `Ctrl+A` 后可批量平移、删除节点，`Ctrl+Z`/`Ctrl+Shift+Z` 撤销或重做。
- 虚线绿色框是 `300 mm × 300 mm` 车体中心可移动范围，即图纸坐标 `X/Y=150～2250 mm`；越界方案不能保存或播放。

方案 JSON 使用 `map_version: 2`。`start` 保存官方图纸坐标和起始朝向，`commands` 保存标定后固定世界坐标系中的顺序 GOTO Pose 命令；旧版方案不再兼容。

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
