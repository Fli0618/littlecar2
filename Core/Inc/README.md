# Core/Inc 目录说明

世界坐标完整位姿要求 OPS 位置与当前选中航向均有效且未超时；调试 flags 中 `POSE_FRESH` 和 `YAW_FRESH` 分别表示两类时间戳的新鲜度。`YAW_FRESH` 始终将当前航向源的有效位与其独立更新时间戳配对判断，不改变调参遥测协议。

`advance_test.h` 声明现场调试测试入口，包括仅手动调用的 `Test_MMCL()` 和 `AdvanceTest_VerifyYawSourceFreshness()`。前者会驱动当前底盘四个电机，使用前必须确认车辆架空并具备急停条件；后者只在非活动运动状态下切换 OPS/WIT 航向源并输出新鲜度标志，不发送底盘运动命令。

本目录保存 STM32 业务模块的公共头文件。`advance_control.h` 定义底盘控制权，`advance_motion.h` 定义位姿外环运行时 PID 配置、版本化提交和读取接口；`comm_jetson.h` 定义 USART6 Jetson 视觉服务接口、检测结果和默认发送周期；`comm_tuner.h` 定义 USART1 在线调参协议的初始化、前台解析、心跳超时和 HAL 回调转发接口。

所有周期接口统一使用 `*_Update()` 命名并仅由 TIM6 调用。通信模块不控制底盘；二维码 Blocking 接口等待期间只检查状态并执行 `__WFI()`。
`advance_motion.h` 提供 `AdvanceMotion_FollowPathEx()` 异步连续路径接口；调用方必须在任务结束前保持路径数组有效且不修改，中间采样点不会触发停车。
`AdvanceMotion_FollowPathEx()` 强制关闭单点 Goto 的大航向先对准策略。路径中段使用动态前视切向前馈、投影横向 PD、插值航向 PD 和航向变化率前馈；接近终点且参考/实测速度均降到捕获阈值后，才切换到完整 Goto PID 精确停车。
调试快照中的 `nearest_segment_index` 是投影点所在段，`target_segment_index` 是前视点所在段；路径进度与剩余量均为累计弧长，单位为 mm。
