# littlecar2 Jetson 视觉服务

`main.py` 是单线程常驻视觉服务。它打开串口与摄像头，非阻塞解析 STM32 指令；START 后连续执行对应高级检测并按指定周期上报最新结果，STOP 后立即停发但不退出服务或卸载模型。

## 配置与运行

在 `main.py` 顶部修改 `SERIAL_PORT`、`SERIAL_BAUDRATE`、`CAMERA_ID`、`DEFAULT_PERIOD_MS`。默认周期为 40 ms，串口默认 `/dev/ttyTHS1`、115200。

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python main.py
conda run -n low_numpy python -m pytest tests -q
```

模型由现有缓存按权重路径复用，START/STOP 只重置高级跟踪状态。协议字段与会话规则见 [STM32 协议文档](../MDK-ARM/docs/上下位机通信协议.md)。
