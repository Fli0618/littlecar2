# Core/Src 目录说明

`sensor_limit.c` 只读取 `LIFT_UP_LIMIT_GPIO_Port`/`LIFT_UP_LIMIT_Pin`，为升降轴顶部归零提供原始电平和有效状态接口；其余 CubeMX 限位宏不进入业务逻辑。`advance_arm.c` 使用该接口完成阻塞式归零，并以反馈位置和 `ARM_LIFT_POS_MAX` 实施升降软件行程保护。

`AdvanceWorld_ResetOrigin()` 在 OPS 航向模式仅依赖新鲜 OPS 数据，在 WIT 航向模式额外要求新鲜 WIT 航向；重置先使用局部候选状态完成计算，失败不会清除已建立的原点或世界位姿。`advance_motion.c` 对位置、位姿和路径控制同时校验 OPS 位置与当前航向时间戳，且当前航向的有效位与 OPS/WIT 各自的更新时间戳成对读取，航向超时时进入安全停止。

`advance_test.c` 包含现场调试入口。`Test_MMCL()` 会使用当前底盘四个电机验证多电机速度命令的批量装载、广播发送、同步启动、正反转、停止和失能流程。调用前必须将车辆架空，并准备断电或急停措施；该函数不会自动接入启动流程。`AdvanceTest_VerifyYawSourceFreshness()` 仅在运动控制非活动时切换 OPS/WIT 航向源，等待既有周期任务刷新快照后输出 `POSE_FRESH`、`YAW_FRESH` 与 `VALID` 标志，并恢复原航向源。

本目录保存 STM32 源文件和 HAL 回调入口。`comm_jetson.c` 负责 USART6 DMA + IDLE 接收、CRC 校验、SESSION 过滤、命令发送、结果缓存和二维码等待超时；`comm_stdio.c` 负责禁用 ARMCC5 semihosting 并将 `printf` 重定向到 USART1；`comm_tuner.c` 负责 USART1 DMA + IDLE 的二进制调参协议、PID/位姿命令分发、响应队列和心跳超时停车；`main.c` 负责初始化、TIM6 周期入口和 HAL 回调分发。

USART1 的两种用途互斥：普通模式允许 `printf` 使用阻塞发送，且不得在 TIM6 或其他中断回调中调用；`ONLINE_DEBUG_MODE=1` 时由 PID 调参协议独占 USART1，stdio 重定向仍然有效但字符会被静默丢弃，避免 ASCII 日志污染二进制帧。

TIM6 每 1 ms 推进通信状态，按 10 ms 更新传感器与世界位姿，按 20 ms 更新电机通信并检查位姿外环 PID 的待生效配置；位姿外环仅在 WORLD 控制权下输出速度，视觉控制仅在 VISUAL 控制权下输出速度。视觉阻塞接口在自身内部每 20 ms 调度私有控制步骤，不再由 TIM6 或主循环调用；其余后台任务仍由 TIM6 统一推进。主循环不消费任务标志，也不主动调用任何后台 Update。

`comm_tuner.c` 在在线调参模式下自启动后每 40 ms 持续发送遥测帧，空闲、GOTO 运行、完成、STOP 和心跳超时后的状态均可被上位机观察。空闲快照会每 20 ms 刷新最新世界位姿，使手动拖拽也能反映 OPS 位置与 WIT 优先航向。世界 yaw 在 STM32 首次取得有效传感器数据时归零，后续控制与遥测均使用该相对角度。遥测不改变远程 GOTO 的心跳超时停车保护；GOTO 线速度上限为 1500 mm/s。

GOTO 仅需由上位机发送一次，STM32 的 `AdvanceMotion_Update()` 会在后续 20 ms 周期内持续推进到达、运动超时、位姿失效或 STOP 等终态。在线调参模式额外要求 GUI 后台持续发送心跳；心跳丢失 1500 ms 后会安全停车。遥测保留字段会报告远程目标是否活动、最近心跳年龄及是否发生心跳超时，便于区分安全停车与 PID 控制问题。
`advance_motion.c` 在同一个 20 ms `AdvanceMotion_Update()` 中执行单点与连续路径控制。路径模式在有限前向窗口中更新单调投影进度，并沿折线弧长插值得到动态前视点；前方曲率、航向梯度和末端制动距离共同限制参考速度，参考速度与前视距离均按周期做变化率限制。当前投影曲率在密集采样曲线上按相邻顶点连续插值；稀疏折线的顶点曲率前馈仅在拐点局部生效，避免整条长直线被后续拐点提前横向推离。
路径中段不复用到点平移 PID：前视段切向前馈负责前进，当前投影段法向 PD 负责回到路径，投影进度插值 yaw 与航向变化率前馈共同产生角速度。横向误差超过 20 mm 后参考速度连续下降；横向误差达到 50 mm 并持续 60 ms 时进入 `OFF_PATH` 终态并平滑停车，对应 400 mm 车道与 300 mm 车宽的理论极限。剩余距离不大于 60 mm 且参考、实测平移速度均不大于 150 mm/s 后，控制器才进入最终 Goto PID 捕获。路径超时按总弧长估算，无进展保护按累计弧长判断。

`advance_holonomic_position.c` 实现轻量全向位置控制器：平移按起点到终点的标量路径规划，航向按最短有符号角度差独立规划，二者复用同一离散梯形/三角轮廓；每个 20 ms 周期生成连续参考位姿与速度后，按 `v_cmd = v_ref + Kp*e_pose + Kv*e_velocity` 做前向/横向/航向独立反馈（无积分项），修正量单独限幅，再经三个对角 scale 校准后调用 `Chassis_SetBodyVelocityEx()`。速度估计使用 OPS/WIT 位姿增量加一阶低通（alpha=0.2）。控制器仅在 `ADVANCE_CONTROL_HOLONOMIC` 控制权下输出速度；到达、超时、位姿异常或取消时先停车再释放控制权。调试入口见 `Test_Holonomic_GotoPoseBlocking()`，实车调试顺序见 `Core/Src/advance_holonomic_position.md`。

全向配置使用 active/pending 双缓冲和 revision：`AdvanceHolonomic_RequestConfig()` 提交后由固定 20 ms 周期应用，`AdvanceHolonomic_GetConfig()` 只返回 active 值。运行中修改 Kp/Kv/scale 在下一周期生效，accel/decel 不重建当前轮廓；scale 后仍执行最终 `vmax/wmax` 限幅。`AdvanceControl_CancelActive()` 会按控制权路由到 WORLD 或 HOLONOMIC，STOP、心跳超时和断链清理保持同一安全路径。
