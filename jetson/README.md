# littlecar2 Jetson 视觉服务

`main.py` 是由 STM32 指令驱动的单线程常驻视觉服务。服务启动时同时打开二维码相机和视觉相机；二维码模式仅使用二维码相机，颜色、圆环和圆盘中心模式仅使用视觉相机。

## 配置与运行

在 `main.py` 顶部配置 `SERIAL_PORT`、`SERIAL_BAUDRATE`、`CAMERA_QR_ID`、`CAMERA_VISION_ID` 和 `DEFAULT_PERIOD_MS`。默认二维码相机为 `0`，视觉相机为 `1`。

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python main.py
conda run -n low_numpy python -m pytest tests -q
```

二维码命令为 `CMD_START_QR = 0x05`，结果命令为 `CMD_QR_RESULT = 0x84`。二维码结果 Payload 只包含任务码本身，必须严格为 15 个 ASCII 字节，例如 `156+123+516+231`；不包含长度、结束符、状态或换行。

高级二维码检测在最近 5 帧中确认同一码至少 3 次，仅在首次确认、任务码变更或已消失任务码再次出现时上报一次。短暂漏检不会解除锁存，连续 5 帧未识别到合法任务码后才重新布防。

可使用以下脚本检查相机和逐帧状态，脚本不创建 GUI，按 Ctrl+C 正常退出：

```powershell
conda run -n low_numpy python scripts/qr_advance_test.py
```
