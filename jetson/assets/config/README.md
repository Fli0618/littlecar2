# HSV 配置目录

`hsv_colors.json` 是 HSV 颜色分类的默认配置。服务仅将其作为颜色规则来源，不包含相机、串口或模型配置。

顶层 `version` 当前固定为 `1`。`colors` 以颜色名为键，每项包含 STM32 颜色编号 `type`、开关 `enabled` 和一个或多个 HSV 闭区间 `ranges`。红色可使用两个区间覆盖色相环两端。色相范围为 `0..179`，饱和度与明度范围均为 `0..255`。

`sampling` 规定检测框中心椭圆采样比例与最小判定阈值：`scale_x`、`scale_y`、`min_coverage` 和 `min_margin` 位于 `0..1`；`min_pixels` 为非负整数。`processing` 的三个卷积核尺寸只能为 `0` 或正奇数；形态学迭代次数为非负整数。

请使用 `vision.hsv_color.save_hsv_config` 保存新的配置。该函数会先校验完整结构，再通过同目录临时文件原子替换目标文件；手工修改后也应执行 HSV 单元测试确认配置可读取。
