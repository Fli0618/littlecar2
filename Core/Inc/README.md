# Core/Inc 目录说明

世界坐标完整位姿要求 OPS 位置与当前选中航向均有效且未超时；调试 flags 中 `POSE_FRESH` 和 `YAW_FRESH` 分别表示两类时间戳的新鲜度。`YAW_FRESH` 始终将当前航向源的有效位与其独立更新时间戳配对判断，不改变调参遥测协议。

`advance_test.h` 声明现场调试测试入口，包括仅手动调用的 `Test_MMCL()` 和 `AdvanceTest_VerifyYawSourceFreshness()`。前者会驱动当前底盘四个电机，使用前必须确认车辆架空并具备急停条件；后者只在非活动运动状态下切换 OPS/WIT 航向源并输出新鲜度标志，不发送底盘运动命令。

本目录保存 STM32 业务模块的公共头文件。`advance_control.h` 定义底盘控制权，`advance_motion.h` 定义位姿外环运行时 PID 配置、版本化提交和读取接口；`comm_jetson.h` 定义 USART6 Jetson 视觉服务接口、检测结果和默认发送周期；`comm_tuner.h` 定义 USART1 在线调参协议的初始化、前台解析、心跳超时和 HAL 回调转发接口。

所有周期接口统一使用 `*_Update()` 命名并仅由 TIM6 调用。通信模块不控制底盘；二维码 Blocking 接口等待期间只检查状态并执行 `__WFI()`。
`advance_motion.h` 提供 `AdvanceMotion_FollowPathEx()` 异步连续路径接口；调用方必须在任务结束前保持路径数组有效且不修改，中间采样点不会触发停车。
`AdvanceMotion_FollowPathEx()` 强制关闭单点 Goto 的大航向先对准策略。路径中段使用动态前视切向前馈、投影横向 PD、插值航向 PD 和航向变化率前馈；接近终点且参考/实测速度均降到捕获阈值后，才切换到完整 Goto PID 精确停车。
连续路径的 14 项控制与速度规划参数由 `AdvanceMotion_PathControlConfig_t` 成组保存，可读取、提交或恢复默认；待应用组只在 20 ms 控制周期边界切换，活动路径仅约束当前前视范围，不重置路径进度。
调试快照中的 `nearest_segment_index` 是投影点所在段，`target_segment_index` 是前视点所在段；路径进度与剩余量均为累计弧长，单位为 mm。

## 运动默认配置

`advance_motion_config.h` 保存由工作台导出的 6 项单点 PID、20 项路径控制参数和 1 项 GOTO 默认策略，供 `advance_motion.c` 构建编译期默认值。该文件可整体替换；`advance_motion.h` 仅保留公共类型、约束和接口。路径参数此前提到的 14 项已废止，当前结构与调参协议均为 20 项。

单点 GOTO 的 21 项分阶段速度规划参数由 `AdvanceMotion_GotoControlConfig_t` 成组管理，使用 active/pending/revision 在 20 ms 控制周期边界原子切换。其只影响 `AdvanceMotion_GotoPoseEx()` 的平移、航向独立加减速与末段捕获，不改变 `AdvanceMotion_FollowPathEx()`、路径上下文或既有 GOTO 载荷。
