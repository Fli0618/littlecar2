# 底盘运动调试工作台

工作台以 upstream Workbench 为基线，使用 `QStackedWidget` 在遥测图表和 `MapEditorWidget` 之间切换。PID、全向位置、单点、路径和连接面板通过同一个 `SessionController` 通信，地图方案同步、完整方案执行和路径上传仍由原有控制器负责。

## 参数热加载

“全向位置”页提供 12 项运行时参数：三项轮廓加速度、六项 Kp/Kv 和三项 scale。读取、应用、恢复均使用协议 V3 的 `0x29` 至 `0x2B` 命令；应用和恢复会轮询板端 active revision，只有确认在 20 ms 周期边界生效后才显示“已生效”。旧固件对 GET 返回 `BAD_COMMAND` 时仅禁用全向页，串口、PID、路径和地图功能保持可用。

单点页保留经典位置 PID 与全向位置控制器选择器。STOP、心跳超时和断开连接统一经控制权路由执行，路径控制器不绕过 `SessionController`。

## 统一导出

只有串口已连接且 PID、路径、GOTO 策略、全向参数四类状态都由板端确认后，才启用“导出固化参数”。导出对话框只生成一个只读 `advance_motion_config.h`，包含四类 revision 和全部宏；不会再生成全向独立头文件。

## 安装与启动

```powershell
conda run -n low_numpy pip install -e tools/pid_tuner
conda run -n low_numpy pip install -e tools/map_planner
conda run -n low_numpy pip install -e tools/motion_workbench
conda run -n low_numpy motion-workbench
```

地图执行需要先完成起点标定和坐标同步。若板端有效遥测已经位于零点容差内，工作台会直接恢复同步；否则仍需执行 `ResetOrigin`。工作台 STOP 不能替代机械急停；实车调试必须有人监护。
