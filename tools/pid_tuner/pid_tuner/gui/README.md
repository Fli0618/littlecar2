# GUI 模块

该目录实现 PySide6 与 pyqtgraph 调参界面。串口协议仅通过上层 `SerialClient` 访问，Qt 主线程以 40 ms 定时器批量刷新曲线。

## 公共控件

`widgets.py` 提供不依赖串口实现的公共组件：

- `PidControlPanel`：PID 参数编辑、读取、应用和恢复请求。
- `ConnectionMotionPanel`：串口连接、航向源、零点重置与组合、位置、角度三类单点 GOTO 请求。

两个组件只发出 Qt 信号；`MainWindow` 负责将信号连接至 `SessionController`，并协调方案存储、遥测缓冲和图表。因此其他工具可以复用控件，而无需复制 `SerialClient`、协议代码或遥测绘图组件。

## 心跳保活

GOTO 确认后，GUI 以 250 ms 周期发送心跳。心跳请求在途时不会重复排队；STOP、运动终态、断开连接或心跳失败会停止保活。

## 分辨率适配

控制区和图表区各自使用滚动容器。窗口尺寸不足时保持控件与图表的最小可读尺寸，通过滚动查看完整内容。
