# Jetson vision 目录

`advance_detect_color` 与 `advance_detect_circle` 返回经过多帧确认的目标；每个目标保留 `type`、中心、`bbox`（`[x1, y1, x2, y2]`）、置信度、`measured` 和 `support_count`。未测量时可保留卡尔曼预测，但下游不能只按预测判定到达；预测期间 `bbox` 会随预测中心平移。

视觉服务中，颜色物料任务使用相机画面上方 3/4 作为模型输入，同心圆任务使用完整画面。颜色任务的检测结果会恢复为原始 640×480 相机坐标后再发送给 STM32；通信协议和坐标定义不变。

正式颜色检测由 `main.py` 中的 `COLOR_DETECTION_BACKEND = "yolo_hsv"` 控制：YOLO 只提供候选框和中心点，`classify_bbox_hsv()` 在候选框内判断颜色，随后再进入跟踪。正式混合链路不执行霍夫圆检测、重新定位或其他候选搜索；不确定的普通候选默认丢弃，`type=6` 的 EmptySlot 保留。`"yolo"` 可用于纯 YOLO 调试，`"hsv"` 仅保留旧霍夫 HSV 接口兼容，不作为正式混合链路。

`advance_detect_disk_center` 返回盘中心、支持点数与 `measured_count`。零支持点返回 `NO_TARGET` 和 `(0, 0)`。`reset_advance_tracking()` 是公开接口，视觉服务在 START 与 STOP 时调用。模型仍在当前进程内缓存，避免重复加载。

`preview.py` 的 ASCII 标注继续由 OpenCV 绘制；中文状态文本通过 Fontconfig
查找包含中文字形的系统字体，并使用 Pillow 局部蒙版叠加。Jetson 可执行
`fc-match ':lang=zh-cn:charset=4e2d'` 检查实际匹配字体。Fontconfig 不可用时
回退到 Pillow 默认字体，预览服务不会因此中断。
