# Jetson 视觉服务协议模块

模块只提供无状态 CRC16-Modbus、帧打包和增量字节流解析。帧为 `5A A5 CMD SESSION LEN PAYLOAD CRC16_L CRC16_H`，CRC 覆盖 `CMD`、`SESSION`、`LEN` 和 Payload。

比赛启动命令为 `0x10`，由 Jetson 使用 session `0` 和空 Payload 打包发送；它不是视觉检测 START 命令，不携带检测周期。

不再提供旧的 `Car` 主控客户端、命令集合、心跳、异常类型或 dataclass 协议对象。START 命令指定检测模式和 `period_ms:uint16`；STOP 立即清空当前模式。完整命令和结果字段见仓库通信协议文档。
