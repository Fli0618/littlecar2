# Jetson 视觉服务协议模块

本目录只保留视觉终端实际使用的最小串口协议：

- `commands.py`：检测启动、停止、比赛启动、ACK 和识别结果命令常量。
- `frame.py`：CRC16-Modbus、帧打包和增量字节流解析。
- `__init__.py`：导出上述最小接口。

帧格式为：

```text
5A A5 CMD SESSION LEN PAYLOAD CRC16_L CRC16_H
```

CRC 覆盖 `CMD`、`SESSION`、`LEN` 和 Payload。视觉 START 命令携带 `period_ms:uint16`，STOP 立即清空当前检测模式。比赛启动命令为 `0x10`，由 Jetson 使用 session `0` 和空 Payload 发送，它不属于视觉检测 START 命令。

Jetson 当前不是底盘主控端，因此不再保留旧的 `Car` 高层客户端、命令组协议、心跳线程、请求/ACK/DATA 传输层、异常类型和协议 dataclass。串口服务编排统一位于 `jetson/main.py`。
