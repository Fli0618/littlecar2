# Core/Src 目录说明

本目录保存 STM32 源文件和 HAL 回调入口。`comm_jetson.c` 负责 USART6 DMA + IDLE 接收、CRC 校验、SESSION 过滤、命令发送、结果缓存和二维码等待超时；`main.c` 负责初始化、TIM6 周期入口和 HAL 回调分发。

TIM6 每 1 ms 推进通信状态，按 10 ms 更新传感器与世界位姿，按 20 ms 更新电机通信并执行当前控制模式。主循环不消费任务标志，也不主动调用任何 Update。
