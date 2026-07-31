# Core/Src 目录说明

`advance_test.c` 包含现场调试入口。`Test_MMCL()` 会使用当前底盘四个电机验证多电机速度命令的批量装载、广播发送、同步启动、正反转、停止和失能流程。调用前必须将车辆架空，并准备断电或急停措施；该函数不会自动接入启动流程。

本目录保存 STM32 源文件和 HAL 回调入口。`comm_jetson.c` 负责 USART6 DMA + IDLE 接收、CRC 校验、SESSION 过滤、命令发送、结果缓存和二维码等待超时；`comm_stdio.c` 负责禁用 ARMCC5 semihosting 并将 `printf` 重定向到 USART1；`comm_tuner.c` 负责 USART1 DMA + IDLE 的二进制调参协议、PID/位姿命令分发、响应队列和心跳超时停车；`main.c` 负责初始化、TIM6 周期入口和 HAL 回调分发。

USART1 的两种用途互斥：普通模式允许 `printf` 使用阻塞发送，且不得在 TIM6 或其他中断回调中调用；`ONLINE_DEBUG_MODE=1` 时由 PID 调参协议独占 USART1，stdio 重定向仍然有效但字符会被静默丢弃，避免 ASCII 日志污染二进制帧。

TIM6 每 1 ms 推进通信状态，按 10 ms 更新传感器与世界位姿，按 20 ms 更新电机通信并检查位姿外环 PID 的待生效配置；位姿外环仅在 WORLD 控制权下输出速度，视觉控制仅在 VISUAL 控制权下输出速度。主循环不消费任务标志，也不主动调用任何 Update。

`comm_tuner.c` 在在线调参模式下自启动后每 40 ms 持续发送遥测帧，空闲、GOTO 运行、完成、STOP 和心跳超时后的状态均可被上位机观察。空闲快照会每 20 ms 刷新最新世界位姿，使手动拖拽也能反映 OPS 位置与 WIT 优先航向。世界 yaw 在 STM32 首次取得有效传感器数据时归零，后续控制与遥测均使用该相对角度。遥测不改变远程 GOTO 的心跳超时停车保护；GOTO 线速度上限为 1500 mm/s。
