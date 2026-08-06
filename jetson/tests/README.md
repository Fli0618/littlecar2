# Jetson 测试

`test_protocol_frame.py` 覆盖新视觉服务帧的 CRC、半帧、粘包、垃圾字节、STOP、周期解析和 SESSION 切换。`test_vision.py` 覆盖高级跟踪重置、支持计数、预测标志、物料盘 `measured_count`、零支持点和模型缓存。
`test_competition_gui.py` 覆盖 Tk 中文字体选择与默认字体回退；`test_preview.py`
覆盖 Fontconfig 字体解析、ASCII/Unicode 绘制分流和边界裁剪。

```powershell
conda run -n low_numpy python -m pytest tests -q
```

`test_hybrid_color.py` 覆盖 YOLO 候选框的 HSV 修正、不确定策略、EmptySlot 与跟踪调试字段；`test_protocol_client.py` 同时覆盖颜色后端选择、物料盘纯 YOLO 和目标 Payload 兼容性。
