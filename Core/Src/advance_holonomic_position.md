# 全向位置控制器（AdvanceHolonomic）调试说明

模块：`Core/Src/advance_holonomic_position.c`。
调试入口：`Test_Holonomic_GotoPoseBlocking()`，默认相对当前位置前进 300 mm 并保持当前航向，方向、距离与参数按实车调试需要临时修改。

实车调试必须按以下顺序进行，不要一开始同时调整全部参数。

1. 关闭反馈验证基础运动方向：临时将六个反馈增益设为 0（kp/kv × forward/lateral/yaw），仅验证参考速度输出与方向，覆盖前进、后退、左移、右移、顺时针、逆时针。
2. 标定三个 scale：分别记录命令速度与 OPS 实测稳定速度，得到 forward_scale、lateral_scale、yaw_scale；优先修复机械方向与安装问题，不用 scale 掩盖明显机械故障。
3. 设置运动轮廓：标定 linear_accel_mm_s2、linear_decel_mm_s2、yaw_accel_deg_s2；先低速确认停止距离，再逐步提高 goal.vmax_mm_s 与 goal.wmax_deg_s。
4. 调位置反馈：只调 kp_forward、kp_lateral、kp_yaw，使位置偏差收敛且不产生明显振荡。
5. 调速度反馈：逐步增加 kv_forward、kv_lateral、kv_yaw，用于抑制速度滞后、过冲和末端余速。

默认参数均为保守初值，需实车标定；`AdvanceHolonomic_SetConfig()` 仅在控制器空闲（非 RUNNING/SETTLING）时生效。
