# 比赛显示界面

`competition_gui.py` 提供基于 Tkinter 的普通比赛窗口，只负责显示和用户点击事件。

界面不直接访问串口、摄像头或视觉模型；这些资源仍由 `jetson/main.py` 统一管理。
