# map_planner 包

`models.py` 定义方案和节点数据，`geometry.py` 提供坐标变换，`sim.py` 提供离线仿真，`storage.py` 负责 JSON 方案存储，`gui.py` 负责 PySide6 地图编辑界面。

固定地图图元仅用于视觉标注，不参与路径碰撞判定；越界检查只使用 `2400 mm × 2400 mm` 场地外框和 `300 mm` 车体尺寸。
