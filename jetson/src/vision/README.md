# Jetson vision 目录

`advance_detect_color` 与 `advance_detect_circle` 返回经过多帧确认的目标；每个目标保留 `class_id`、中心、置信度、`measured` 和 `support_count`。未测量时可保留卡尔曼预测，但下游不能只按预测判定到达。

`advance_detect_disk_center` 返回盘中心、支持点数与 `measured_count`。零支持点返回 `NO_TARGET` 和 `(0, 0)`。`reset_advance_tracking()` 是公开接口，视觉服务在 START 与 STOP 时调用。模型仍在当前进程内缓存，避免重复加载。

`preview.py` 的 ASCII 标注继续由 OpenCV 绘制；中文状态文本通过 Fontconfig
查找包含中文字形的系统字体，并使用 Pillow 局部蒙版叠加。Jetson 可执行
`fc-match ':lang=zh-cn:charset=4e2d'` 检查实际匹配字体。Fontconfig 不可用时
回退到 Pillow 默认字体，预览服务不会因此中断。
