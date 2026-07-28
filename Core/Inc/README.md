# Core/Inc 目录说明

本目录保存 STM32 业务模块的公共头文件。`advance_control.h` 定义底盘控制权，`advance_motion.h` 定义位姿外环运行时 PID 配置、版本化提交和读取接口；`comm_jetson.h` 定义 USART6 Jetson 视觉服务接口、检测结果和默认发送周期；`comm_tuner.h` 定义 USART1 在线调参协议的初始化、前台解析、心跳超时和 HAL 回调转发接口。

所有周期接口统一使用 `*_Update()` 命名并仅由 TIM6 调用。通信模块不控制底盘；二维码 Blocking 接口等待期间只检查状态并执行 `__WFI()`。
