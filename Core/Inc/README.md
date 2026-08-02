# Core/Inc 目录说明

`advance_test.h` 声明现场调试测试入口，包括仅手动调用的 `Test_MMCL()`。该测试会驱动当前底盘四个电机，使用前必须确认车辆架空并具备急停条件。

本目录保存 STM32 业务模块的公共头文件。`advance_control.h` 定义底盘控制权，`advance_motion.h` 定义位姿外环运行时 PID 配置、版本化提交和读取接口；`comm_jetson.h` 定义 USART6 Jetson 视觉服务接口、检测结果和默认发送周期；`comm_tuner.h` 定义 USART1 在线调参协议的初始化、前台解析、心跳超时和 HAL 回调转发接口。

所有周期接口统一使用 `*_Update()` 命名并仅由 TIM6 调用。通信模块不控制底盘；二维码 Blocking 接口等待期间只检查状态并执行 `__WFI()`。
`advance_motion.h` 提供 `AdvanceMotion_FollowPathEx()` 异步连续路径接口；调用方必须在任务结束前保持路径数组有效且不修改，中间采样点不会触发停车。
`AdvanceMotion_FollowPathEx()` 强制关闭单点 Goto 的大航向先对准策略，确保路径中间段平移与航向 PID 并行；路径参考更新后才读取当前点的约束标志。
