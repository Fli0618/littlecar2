# map_planner 包

## v7 流程模型

## 工作台嵌入

`gui.MapEditorWidget` 是可嵌入的地图编辑组件，复用原有的 `MapView`、静态地图层、规划层与坐标转换模型。工作台通过 `canvas` 取得画布，使用 `set_plan()` / `get_plan()` 交换方案，使用 `select_candidate()` 选择当前流程项，并通过 `set_runtime_pose()` / `clear_runtime_pose()` 显示运行时车辆覆盖层。

组件提供 `plan_changed(Plan)`、`candidate_selected(int)` 和 `runtime_overlay_changed(Pose | None)` 信号。独立程序仍由 `PlannerWindow` 承载该组件，保留方案管理、仿真与代码生成功能。

`Plan.steps` 是唯一的运行时和持久化流程接口。`ContinuousPathSegment.points[0]` 为锁定入口点，后续点包含软途经点和最终停车点；旧版 `mode`、`waypoints`、`path_points` 与 v5/v6 迁移已移除。

`models.py` 定义方案、停点动作、原地转向和连续路径段；`geometry.py` 提供坐标变换，`sim.py` 提供停点近似仿真和连续路径几何播放，`storage.py` 负责 JSON 方案存储，`gui.py` 负责 PySide6 地图编辑界面，`codegen_c.py` 按步骤顺序生成 STM32 代码。

`step_turn.py` 负责将 `StepTurnPathSegment` 的用户折线编译为执行用的 A/B 点，原始拐点 C 不进入执行路径；`analyze_step_turn_path()` 和 `generate_step_turn_path_points()` 是公共编译入口。`path_materializer.py` 将连续、贝塞尔和垫步路径统一实体化，供仿真、上传和代码生成复用，不修改原始方案。

地图编辑仅操作 `Plan.steps`。选择模式支持框选、Ctrl 追加选择和 Ctrl+A；批量平移仅作用于 `Waypoint` 步骤，批量删除按步骤索引从后向前执行，并同步后续连续段的入口点。

## 代码生成模式

“生成 STM32 业务函数”窗口默认使用“严谨反馈”模式：每个阻塞运动调用都会检查到达状态，失败时取消运动并退出任务函数。左下角按钮可切换为“开环忽略结果”模式；该模式仍使用阻塞 GOTO 与连续路径接口保证步骤顺序，但业务代码不检查到达、超时或取消结果，也不会生成 `AdvanceMotion_Cancel()`。

方案 JSON 使用 `map_version: 7`，仅持久化有序的 `steps`：`goto_pose`、`rotate_in_place` 和 `continuous_path`。连续路径段的第一个点为入口点，必须与上一动作的终点一致；代码生成会依次调用 GOTO、原地转向（保持当前位置的 GOTO）和 `AdvanceMotion_FollowPathBlocking`。

固定地图图元仅用于视觉标注，不参与路径碰撞判定；越界检查只使用 `2400 mm × 2400 mm` 场地外框和 `300 mm` 车体尺寸。
