# map_planner 包

`models.py` 定义方案、停点动作和连续路径位姿点，并维护 v5 到 v6 的迁移；`geometry.py` 提供坐标变换，`sim.py` 提供停点近似仿真和连续路径几何播放，`storage.py` 负责 JSON 方案存储，`gui.py` 负责 PySide6 地图编辑界面，`codegen_c.py` 按模式生成 STM32 代码。

方案格式 v6 将起点图纸坐标与世界坐标路径分开保存。`stop_point` 模式沿用逐节点线速度、角速度、航向、停止、停留和超时参数；`continuous` 模式使用 `PathPosePoint` 几何位姿点，播放不表示实车仿真。

固定地图图元仅用于视觉标注，不参与路径碰撞判定；越界检查只使用 `2400 mm × 2400 mm` 场地外框和 `300 mm` 车体尺寸。
