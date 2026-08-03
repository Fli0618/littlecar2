# 协议模块

此目录包含正式 CLI 的可复用核心：

- `models.py`：PID、连续路径控制参数、运动目标、遥测和客户端错误类型。
- `protocol.py`：STM32 二进制帧编解码与字节流恢复。
- `serial_client.py`：单串口后台接收线程、请求重试与遥测分发。

模块不依赖 GUI；测试可向 `SerialClient` 注入 fake transport，避免访问真实 COM 口。

协议 V2 使用 `0x26/0x27/0x28` 读取、设置和恢复连续路径参数，响应 `0x86` 携带 32 位修订号与 14 个 `float` 参数。
