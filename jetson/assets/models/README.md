# assets/models 目录说明

本目录存放视觉模型文件。

- `6color-circle-v3.pt`：彩色物料与 `EmptySlot` 检测模型。
- `circle-with-number-v3.pt`：带数字同心圆检测模型。
- `6color-circle-v3.engine`：由对应 PT 模型在当前 Jetson 环境导出的 TensorRT engine。
- `circle-with-number-v3.engine`：由对应 PT 模型在当前 Jetson 环境导出的 TensorRT engine。
- `RGB_circle.pt` 和 `RGB_circle.onnx`：历史实验模型，不作为正式视觉接口使用。

TensorRT engine 必须在目标 Jetson 上生成，依赖当前 GPU、JetPack、CUDA 和 TensorRT 版本，不能保证跨设备复用。使用 `scripts/tensorrt推理测试/export_models.py` 生成，导出参数为固定 `640x640` 输入、`batch=1`、FP16 和 CUDA device 0。脚本使用 JetPack 自带的 `trtexec`，workspace 限制为 256 MiB、builder optimization level 为 0，以降低 Orin Nano 转换时的内存峰值。

主服务通过 `main.py` 的 `MODEL_BACKEND` 选择 `pt` 或 `engine`。选择 `engine` 时，如果任一 engine 缺失或不兼容，程序会直接报错，不会自动退回 PT。
