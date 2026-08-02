# map_planner 包

## v7 流程模型

`Plan.steps` 是唯一的运行时和持久化流程接口。`ContinuousPathSegment.points[0]` 为锁定入口点，后续点包含软途经点和最终停车点；旧版 `mode`、`waypoints`、`path_points` 与 v5/v6 迁移已移除。

`models.py` 定义方案、停点动作、原地转向和连续路径段；`geometry.py` 提供坐标变换，`sim.py` 提供停点近似仿真和连续路径几何播放，`storage.py` 负责 JSON 方案存储，`gui.py` 负责 PySide6 地图编辑界面，`codegen_c.py` 按步骤顺序生成 STM32 代码。

方案 JSON 使用 `map_version: 7`，仅持久化有序的 `steps`：`goto_pose`、`rotate_in_place` 和 `continuous_path`。连续路径段的第一个点为入口点，必须与上一动作的终点一致；代码生成会依次调用 GOTO、原地转向（保持当前位置的 GOTO）和 `AdvanceMotion_FollowPathBlocking`。

固定地图图元仅用于视觉标注，不参与路径碰撞判定；越界检查只使用 `2400 mm × 2400 mm` 场地外框和 `300 mm` 车体尺寸。
