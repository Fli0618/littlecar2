# Jetson src 目录

- `vision/`：YOLO 检测、高级跟踪与物料盘中心估算。
- `protocol/`：STM32—Jetson 视觉服务的函数式帧打包、CRC 和增量解析。
- `ui/`：Tkinter 比赛显示窗口及其纯显示接口。

这里不包含 Jetson 主控 STM32、心跳客户端、后台发送线程或复杂通信封装。服务编排只在 `jetson/main.py` 中完成。
