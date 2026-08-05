# 全向位置控制器主机侧测试

本目录包含 `advance_holonomic_position` 模块的主机侧单元测试，仅在 PC 上编译运行，不加入 EIDE/MDK 固件工程，不依赖真实串口或车辆硬件。

## 运行方式

```powershell
powershell -ExecutionPolicy Bypass -File build_and_run.ps1
```

## 覆盖范围

- 解析梯形/三角形轮廓：平移 100/500/1000 mm、航向 30°/90°/-90°、±180° 附近最短角、零距离、负目标方向；
- 检查初始零值、三角形无巡航段、梯形含巡航段、阶段交界位置/速度连续、最终位置等于目标、最终速度归零、finished 置位；
- `Update` 在 `now_tick == last_update_tick` 时不推进轮廓、不更新时间戳、不下发底盘；
- 连续两次任务启动后，第二个任务首个控制周期前调试快照的修正量与命令字段为零。

测试通过 `#include` 被测 `.c` 文件并注入 HAL/底盘/世界/控制权桩实现；`stubs/` 下提供 `main.h`、`advance_control.h`、`advance_chassis.h` 阴影头以屏蔽 STM32 HAL 依赖，`ADVANCE_HOLONOMIC_UNIT_TEST` 宏由构建脚本定义。
