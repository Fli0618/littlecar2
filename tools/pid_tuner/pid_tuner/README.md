# 协议模块

此目录包含正式 CLI 的可复用核心：

- `models.py`：PID、运动目标、遥测和客户端错误类型。
- `protocol.py`：STM32 二进制帧编解码与字节流恢复。
- `serial_client.py`：单串口后台接收线程、请求重试与遥测分发。

模块不依赖 GUI；测试可向 `SerialClient` 注入 fake transport，避免访问真实 COM 口。
