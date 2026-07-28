# GUI 模块

该目录实现 PySide6 与 pyqtgraph 调参界面。串口协议仅通过上层 `SerialClient` 访问；Qt 主线程以 40 ms 定时器批量刷新曲线。
