# Core/Inc 目录说明

本目录保存 STM32 业务模块的公共头文件。`advance_control.h` 管理底盘控制权，`advance_motion.h` 提供经典世界坐标运动控制，`advance_holonomic_position.h` 提供轻量全向位置控制，`comm_tuner.h` 定义 USART1 在线调参协议入口。

通信、传感器、世界位姿和底盘等后台周期接口由 TIM6 调度；视觉阻塞接口在内部以 20 ms 周期自调度 `AdvanceVisual_Update()`，不依赖 TIM6。通信模块不直接控制底盘；运动模块的阻塞接口只等待状态并使用 `__WFI()`，不会绕过控制权状态机。

## 统一固化参数

`advance_motion_config.h` 是工作台唯一导出目标，按固定顺序包含：6 项单点 PID、20 项路径控制参数、1 项 GOTO 默认策略和全向位置控制器的 12 项运行时默认参数。文件只有一个 include guard 和最终 `#endif`，浮点字面量使用有限的 float32 兼容形式。

`advance_holonomic_position_config.h` 只保留全向控制器的固定安全参数、校验范围、状态机常量和非运行时默认值，并通过 include 引入统一参数头。运行时配置由 `AdvanceHolonomic_Config_t` 的 12 个字段管理，使用 active/pending/revision 在 20 ms 周期边界整体切换。
