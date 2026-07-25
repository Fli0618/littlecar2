# Core/Src 目录说明

本目录保存 STM32 源文件和 HAL 回调入口。`comm_jetson.c` 负责 USART6 DMA + IDLE 接收、CRC 校验、SESSION 过滤、命令发送及最新视觉结果缓存；`main.c` 仅转发 USART6 回调并维护简单视觉阶段。

视觉通信不修改运动参数。视觉阶段启动时取消已有 `AdvanceMotion` 目标，数据超过 200 ms 未刷新时停车并结束该检测阶段。
