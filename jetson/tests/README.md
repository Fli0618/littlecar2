# Jetson 测试

`test_protocol_frame.py` 覆盖新视觉服务帧的 CRC、半帧、粘包、垃圾字节、STOP、周期解析和 SESSION 切换。`test_vision.py` 覆盖高级跟踪重置、支持计数、预测标志、物料盘 `measured_count`、零支持点和模型缓存。
`test_competition_gui.py` 覆盖 Tk 中文字体选择与默认字体回退；`test_preview.py`
覆盖 Fontconfig 字体解析、ASCII/Unicode 绘制分流和边界裁剪。

```powershell
conda run -n low_numpy python -m pytest tests -q
```
