# 全向位置控制器说明

实现位于 `advance_holonomic_position.c`，目标类型复用 `WorldGoalPose2D_t`，控制权由 `ADVANCE_CONTROL_HOLONOMIC` 管理。`AdvanceHolonomic_Update()` 必须在固定 20 ms 周期无条件调用；空闲时刷新快照，运行时推进轮廓和反馈控制。

## 参数热加载

`AdvanceHolonomic_GetConfig()` 读取 active 配置；`AdvanceHolonomic_RequestConfig()` 和 `AdvanceHolonomic_RestoreDefaultConfig()` 写入 pending 配置并返回 revision。下一次 20 ms 周期边界只替换参数，不重建当前目标、速度估计或已经计算的轮廓。运行中修改 Kp/Kv/scale 下一周期生效，accel/decel 只影响下一次 `Start`。

控制命令先乘 forward/lateral/yaw scale，再执行最终 `vmax/wmax` 限幅。取消、超时、断链和心跳超时都必须先停车，再释放全向控制权。

默认运行参数位于 `Core/Inc/advance_motion_config.h`；固定安全阈值和校验范围位于 `advance_holonomic_position_config.h`。实车标定前使用保守值，并确认机械急停可用。
