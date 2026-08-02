# 坐标系与 GotoPose 使用说明

> 2026-07-31 校准：WIT 航向保持原始符号，`ADVANCE_WORLD_WIT_YAW_REVERSED=0`。此前低速闭环日志中角速度指令与测量值持续反号，形成航向正反馈。

## 1. 文档定位

本文说明当前下位机中 world 坐标系、车体坐标系、yaw 正方向、世界速度控制和 `GotoPose` 异步目标点控制的使用方法与运算逻辑。

相关代码：

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| 底盘运动学 | `Core/Inc/advance_chassis.h`、`Core/Src/advance_chassis.c` | 车体速度到四轮 RPM 的麦轮解算 |
| 世界坐标 | `Core/Inc/advance_world.h`、`Core/Src/advance_world.c` | OPS / WIT 数据到 world 位姿的转换 |
| 高级运动 | `Core/Inc/advance_motion.h`、`Core/Src/advance_motion.c` | 世界速度和 GotoPose 状态机 |

## 2. 坐标系约定

### 2.1 world 坐标系

`world` 是软件建立的全局坐标系，不直接等同于 OPS 传感器上电后的原始坐标。

约定如下：

| 量 | 正方向 |
| --- | --- |
| `world +Y` | 软件原点建立时，小车车头朝向 |
| `world +X` | 软件原点建立时，小车右侧方向 |
| `yaw = 0 deg` | 小车车头朝向 `world +Y` |
| `yaw > 0` | 俯视小车，逆时针旋转 |

这样定义后，小车在 yaw 为 0 时，车体坐标和 world 坐标的方向一致：

- 车体向右 = `world +X`
- 车体向前 = `world +Y`
- 逆时针旋转 = yaw 增大

### 2.2 车体坐标系

车体坐标系随车转动，用于底盘速度控制：

| 参数 | 正方向 |
| --- | --- |
| `vx_right_mm_s` | 车体向右 |
| `vy_forward_mm_s` | 车体向前 |
| `wz_ccw_deg_s` | 俯视逆时针旋转 |

底盘物理速度入口：

```c
Chassis_SetBodyVelocityEx(vx_right_mm_s, vy_forward_mm_s, wz_ccw_deg_s, acc);
```

### 2.3 方向配置宏

实车调试时，如果某个方向与约定相反，应优先调整编译期宏，而不是修改控制算法。

单轮方向：

```c
#define CHASSIS_MOTOR_LF_SIGN (1)
#define CHASSIS_MOTOR_RF_SIGN (1)
#define CHASSIS_MOTOR_LR_SIGN (1)
#define CHASSIS_MOTOR_RR_SIGN (1)
```

整车轴向：

```c
#define CHASSIS_BODY_X_SIGN (1)
#define CHASSIS_BODY_Y_SIGN (1)
#define CHASSIS_BODY_WZ_SIGN (1)
```

传感器 yaw 方向：

```c
#define ADVANCE_WORLD_OPS_YAW_REVERSED (0)
#define ADVANCE_WORLD_WIT_YAW_REVERSED (0)
```

调试顺序建议：

1. 先低速确认四个电机 ID 与单轮方向。
2. 再确认 `vx > 0` 是否右移。
3. 确认 `vy > 0` 是否前进。
4. 确认 `wz > 0` 是否俯视逆时针旋转。
5. 最后确认 yaw 增大方向是否为俯视逆时针。

## 3. world 原点建立

OPS 和 WIT 可能保留自身历史读数，因此软件不假设传感器上电后就是零点。

建立 world 原点的入口：

```c
AdvanceWorld_ResetOrigin();
```

调用该函数时，模块会记录当前 OPS 位置、OPS yaw 和 WIT yaw 作为本次运行的软件原点。之后 `AdvanceWorld_GetPoseCopy()` 返回的是相对该软件原点的 world 位姿。

要求：

- 小车应静止。
- OPS 位姿有效。
- WIT yaw 数据有效。
- 建议在实车摆正、传感器稳定后再重置原点。

## 4. OPS / WIT 到 world 位姿

`advance_world` 的核心任务是把传感器原始数据转换为统一 world 位姿。

简化理解：

```text
OPS 原始位置 / OPS 原始 yaw / WIT 原始 yaw
    -> 方向符号修正
    -> 扣除 ResetOrigin 时记录的原点偏移
    -> 得到 world x/y/yaw
```

当前位置输出：

```c
WorldPose2D_t pose;
AdvanceWorld_GetPoseCopy(&pose);
```

`WorldPose2D_t` 中关键字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `x_mm` | mm | 当前 world X |
| `y_mm` | mm | 当前 world Y |
| `yaw_deg` | deg | 当前 world yaw |
| `valid` | 0/1 | 位姿是否有效 |
| `origin_ready` | 0/1 | 软件原点是否建立 |
| `updated_tick` | ms | 位姿更新时间 |

当前实现已提供 OPS 安装补偿配置：`ADVANCE_WORLD_OPS_X_REVERSED`、`ADVANCE_WORLD_OPS_Y_REVERSED`、`ADVANCE_WORLD_OPS_XY_SWAPPED`、`ADVANCE_WORLD_OPS_YAW_OFFSET_DEG`、`ADVANCE_WORLD_OPS_OFFSET_X_MM`、`ADVANCE_WORLD_OPS_OFFSET_Y_MM`。偏移量以底盘旋转中心为基准，软件会先把 OPS 传感器坐标换算为底盘中心坐标，再建立 world 原点。

`WorldPose2D_t` 同时保存位置更新时间 `updated_tick` 与航向更新时间 `yaw_updated_tick`。原点建立时若选用了 WIT yaw，则 WIT 失效会使 world 位姿失效，而不会静默切回 OPS yaw，避免两个零点基准不同导致航向跳变。

## 5. 世界速度到车体速度

上位机发送 world 速度时，希望小车沿固定场地方向运动，而不是沿车头方向运动。

接口：

```c
AdvanceMotion_SetWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc);
```

转换逻辑：

1. 读取当前 world yaw。
2. 将 world 平移速度旋转到车体坐标。
3. 调用 `Chassis_SetBodyVelocityEx()` 输出到底盘。

由于本工程约定 `yaw = 0` 时车头朝 `world +Y`，所以速度变换以车体前方轴为 `+Y`。代码中的 `AdvanceWorld_WorldToBodyVelocity()` 封装了这一步。

直观效果：

| 当前 yaw | 发送 world `+Y` | 车体应执行 |
| --- | --- | --- |
| `0 deg` | 场地向前 | 车体前进 |
| `90 deg` | 场地向前 | 车体向右或向左补偿，取决于当前朝向 |
| `-90 deg` | 场地向前 | 车体向相反侧补偿 |

目标是让运动方向固定在 world 坐标，而不是固定在车头坐标。

## 6. 麦轮速度解算

`Chassis_SetBodyVelocityEx()` 会把车体速度转换为四轮 RPM。

输入：

```text
vx = 车体右移速度，mm/s
vy = 车体前进速度，mm/s
wz = 车体逆时针角速度，deg/s
```

处理步骤：

1. 应用整车方向宏：`CHASSIS_BODY_X_SIGN`、`CHASSIS_BODY_Y_SIGN`、`CHASSIS_BODY_WZ_SIGN`。
2. 使用轮半径、半车长、半车宽、减速比计算四轮目标 RPM。
3. 对四轮混合结果做整体等比例缩放，保证最大绝对 RPM 不超过 `CHASSIS_MAX_RPM`。
4. 应用单轮方向宏 `CHASSIS_MOTOR_*_SIGN`。
5. 通过 `drive_emm` 多电机同步命令发送。

整体等比例缩放很重要。它不会改变四轮之间的比例关系，因此不会像单轮独立截断那样破坏运动方向。

## 7. GotoPose 定位

`GotoPose` 是异步目标点控制。上位机发送目标后，STM32 只在 ACK 中表示“已接收”，不表示“已经到达”。目标是否完成需要查询状态。

代码入口：

```c
AdvanceMotion_GotoPoseEx(&goal, acc);
AdvanceMotion_GetStatus(&status);
AdvanceMotion_Cancel();
```

`main.c`通过 TIM6 每 20 ms 调用 `AdvanceMotion_Update()`，业务层不得主动调用。Blocking 版本在启动目标后只通过 `__WFI()`等待终态；取消操作会释放 `WORLD`控制权并停车。

## 8. GotoPose 目标结构

目标结构为 `WorldGoalPose2D_t`：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `x_mm` | mm | world 目标 X |
| `y_mm` | mm | world 目标 Y |
| `yaw_deg` | deg | 目标 yaw，俯视逆时针为正 |
| `vmax_mm_s` | mm/s | 平移速度上限 |
| `wmax_deg_s` | deg/s | 旋转速度上限 |
| `timeout_ms` | ms | 目标超时，0 表示不启用目标超时 |
| `goal_flags` | bit mask | bit0 启用 yaw 控制，bit1 启用 X/Y 位置控制；至少启用一项 |

默认控制参数在 `advance_motion.h`：

| 宏 | 默认值 | 含义 |
| --- | ---: | --- |
| `ADVANCE_MOTION_CONTROL_PERIOD_MS` | 20 | 控制周期 |
| `ADVANCE_MOTION_DEFAULT_KP/KI/KD_POS` | 0.98 / 0.185 / 0.620 | 位置 PID 默认增益 |
| `ADVANCE_MOTION_DEFAULT_KP/KI/KD_YAW` | 1.42 / 0.625 / 0.427 | yaw PID 默认增益 |
| `ADVANCE_MOTION_POS_TOLERANCE_MM` | 10.0 | 到达位置阈值 |
| `ADVANCE_MOTION_YAW_TOLERANCE_DEG` | 1.5 | 到达 yaw 阈值 |
| `ADVANCE_MOTION_ARRIVE_HOLD_MS` | 150 | 到达保持时间 |
| `ADVANCE_MOTION_POSE_TIMEOUT_MS` | 100 | 位姿超时阈值 |
| `ADVANCE_MOTION_DEFAULT_VMAX_MM_S` | 820.0 | 默认平移速度上限 |
| `ADVANCE_MOTION_DEFAULT_WMAX_DEG_S` | 100.0 | 默认旋转速度上限 |

调用 `AdvanceMotion_GotoPoseEx()` 时，已启用位置约束的目标必须提供大于 0 的 `vmax_mm_s`，已启用 yaw 约束的目标必须提供大于 0 的 `wmax_deg_s`。简化阻塞接口会自动填入默认速度上限。

## 9. GotoPose 控制运算逻辑

状态机每 `ADVANCE_MOTION_CONTROL_PERIOD_MS` 执行一次控制计算。

### 9.1 前置检查

每次控制先检查：

1. 当前是否有活动目标。
2. world 原点是否建立。
3. 当前位姿是否有效。
4. 当前位姿是否超过 `ADVANCE_MOTION_POSE_TIMEOUT_MS`。
5. 是否超过目标 `timeout_ms`。

如果前置条件失败，状态机会停车并进入对应状态：

| 条件 | 状态 |
| --- | --- |
| 原点未建立 | `NO_ORIGIN` |
| 位姿无效或超时 | `NO_POSE` |
| 目标超时 | `TIMEOUT` |
| 主动取消 | `CANCELED` |

### 9.2 位置误差

位置误差在 world 坐标下计算：

```text
error_x = goal.x_mm - pose.x_mm
error_y = goal.y_mm - pose.y_mm
position_error = sqrt(error_x^2 + error_y^2)
```

### 9.3 平移速度 PID 控制

位置环使用运行时 PID 配置生成 world 平移速度。微分项直接使用实测速度，避免对位置误差做数值差分：

```text
vx_world = kp_pos * error_x + ki_pos * integral_x - kd_pos * measured_vx_world
vy_world = kp_pos * error_y + ki_pos * integral_y - kd_pos * measured_vy_world
```

然后按二维向量模长限幅：

```text
speed = sqrt(vx_world^2 + vy_world^2)
if speed > vmax:
    scale = vmax / speed
    vx_world *= scale
    vy_world *= scale
```

这种限幅方式会保持速度方向不变，只降低速度大小。

### 9.4 yaw 误差与旋转速度

只有 `goal_flags` 设置 `ADVANCE_MOTION_GOAL_USE_YAW` 时才控制 yaw。

yaw 误差先 wrap 到 `[-180, 180]`：

```text
yaw_error = wrap(goal.yaw_deg - pose.yaw_deg)
```

再用运行时 yaw PID 生成角速度，微分项使用实测角速度：

```text
wz_ccw = kp_yaw * yaw_error + ki_yaw * integral_yaw - kd_yaw * measured_wz
```

最后限幅到 `wmax_deg_s`：

```text
if wz_ccw > wmax: wz_ccw = wmax
if wz_ccw < -wmax: wz_ccw = -wmax
```

如果没有启用 yaw 控制：

```text
wz_ccw = 0
```

### 9.5 world 速度输出

状态机得到 `vx_world`、`vy_world`、`wz_ccw` 后调用内部世界速度输出：

```text
world velocity
    -> AdvanceWorld_WorldToBodyVelocity()
    -> Chassis_SetBodyVelocityEx()
    -> 四轮 RPM
    -> drive_emm
```

这保证 `GotoPose` 的目标点始终基于 world 坐标，而不是车头坐标。

### 9.6 到达判定

位置到达条件：

```text
position_error <= ADVANCE_MOTION_POS_TOLERANCE_MM
```

若启用 yaw，还需要：

```text
abs(yaw_error) <= ADVANCE_MOTION_YAW_TOLERANCE_DEG
```

首次满足条件时会立即向底盘下发零速度；随后不会立刻进入 `ARRIVED`，而是需要连续保持：

```text
ADVANCE_MOTION_ARRIVE_HOLD_MS
```

这样可以避免位姿短暂抖动导致误判到达。

## 10. 当前运动接口与调度边界

当前仓库没有名为 `CMDSET_CHASSIS` 或 `CHASSIS_GOTO_POSE` 的通用比赛协议；在线调参模式通过 `comm_tuner` 提供独立的 `GOTO_POSE`、停止、状态与遥测命令。固件业务可直接使用本地 C 接口：

```c
AdvanceMotion_GotoPoseEx(&goal, acc);
AdvanceMotion_Update();
AdvanceMotion_GetStatus(&status);
AdvanceMotion_Cancel();
```

`AdvanceMotion_Update()`只能由 TIM6 周期入口调用，控制周期为 `ADVANCE_MOTION_CONTROL_PERIOD_MS`。它从 `AdvanceWorld_GetPoseCopy()` 获取已经由 TIM6 更新的缓存位姿，每个控制周期只读取一次。

需要顺序等待时使用：

```c
AdvanceMotion_GotoGoalBlocking(&goal, acc);
/* 或使用默认约束： */
AdvanceMotion_GotoPoseBlocking(x_mm, y_mm, yaw_deg, acc);
```

该接口启动目标后只检查运动终态并执行 `__WFI()`；传感器、世界位姿、电机 DMA 和超时检查仍由中断异步推进。

## 11. 当前本地使用流程

1. 初始化 OPS、WIT、`CarPose`、`AdvanceWorld`、`AdvanceControl`、`AdvanceMotion` 和 `drive_emm`。
2. 等待 TIM6 更新传感器缓存，并在原点未建立时由周期任务调用 `AdvanceWorld_ResetOrigin()`。
3. 通过 `AdvanceMotion_GotoPoseEx()` 启动异步目标，或使用 `AdvanceMotion_GotoGoalBlocking()`、`AdvanceMotion_GotoPoseBlocking()` 执行顺序流程；连续软途经路径使用 `AdvanceMotion_FollowPathEx()` 或其 Blocking 版本。
4. 通过 `AdvanceMotion_GetStatus()`读取状态；完成、失败、取消或超时后控制权回到 `ADVANCE_CONTROL_NONE`。
5. 不要在业务等待循环中调用任何 `*_Update()`、UART 发送或 `HAL_Delay()`。

## 12. 实车调试建议

建议按以下顺序排查，不要一开始就调整控制算法：

1. 单轮测试，确认电机 ID。
2. 单轮正反方向测试，调整 `CHASSIS_MOTOR_*_SIGN`。
3. 车体右移、前进、逆时针旋转测试，调整 `CHASSIS_BODY_*_SIGN`。
4. 静止重置 world 原点。
5. 旋转小车，确认 yaw 逆时针为正；必要时调整 `ADVANCE_WORLD_*_YAW_REVERSED`。
6. 测试 world `+Y` 在 yaw 为 0、90、-90 度时是否保持场地方向一致。
7. 低速测试 `GotoPose(300, 0)` 和 `GotoPose(0, 300)`。
8. 最后再启用 yaw 控制，测试小角度目标。

如果某项方向相反，优先修改对应方向宏；只有当所有方向宏确认无误后，才考虑物理参数标定或控制参数调整。
