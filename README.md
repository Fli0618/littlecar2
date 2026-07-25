# littlecar2

本仓库包含 STM32F407 小车固件和 Jetson 视觉服务。STM32 是任务主控，Jetson 只负责执行已加载的视觉模型并返回最新检测结果。

## 连续视觉通信

STM32 通过 USART6 发送 START，Jetson 在对应会话内持续检测并按请求周期上报数据；STOP 仅停止当前检测，不退出 Jetson 服务或卸载模型。默认周期为 40 ms。

协议、结果字段、SESSION 过滤和超时规则见 [通信协议](MDK-ARM/docs/上下位机通信协议.md)。固件接口见 [Core 文档](Core/README.md)，Jetson 部署见 [Jetson 文档](jetson/README.md)。
