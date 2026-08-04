#ifndef __ADVANCE_MOTION_H__
#define __ADVANCE_MOTION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "advance_chassis.h"
#include "advance_control.h"
#include "advance_world.h"
#include "main.h"
#include <math.h>

#define ADVANCE_MOTION_PATH_MAX_POINTS ((uint16_t)256U) /*!< 路径协议允许的最大离散点数。 */

/* 连续路径参数。以下保守初值均需要实机标定。 */

/* 路径数据与投影搜索。 */
#define ADVANCE_MOTION_PATH_MIN_SEGMENT_MM (1.0f) /*!< 最小有效线段长度，单位为 mm。 */
#define ADVANCE_MOTION_PATH_SEARCH_SEGMENTS ((uint16_t)12U) /*!< 每周期前向搜索上限，单位为段。 */

/* 参考速度与运动约束。 */
#define ADVANCE_MOTION_PATH_CRUISE_SPEED_MM_S (820.0f) /*!< 巡航速度，单位为 mm/s。 */
#define ADVANCE_MOTION_PATH_MAX_WZ_DEG_S (100.0f) /*!< 路径最大角速度，单位为 deg/s。 */
#define ADVANCE_MOTION_PATH_ACCEL_MM_S2 (800.0f) /*!< 参考速度加速度，单位为 mm/s^2。 */
#define ADVANCE_MOTION_PATH_DECEL_MM_S2 (1000.0f) /*!< 参考速度减速度，单位为 mm/s^2。 */
#define ADVANCE_MOTION_PATH_MAX_LATERAL_ACC_MM_S2 (600.0f) /*!< 横向加速度限值，单位为 mm/s^2。 */

/* 曲率预览与法向前馈。 */
#define ADVANCE_MOTION_PATH_CURVATURE_EPSILON_1_MM (0.00001f) /*!< 曲率除数下限，单位为 1/mm。 */
#define ADVANCE_MOTION_PATH_YAW_GRADIENT_EPSILON_DEG_PER_MM (0.0001f) /*!< 航向梯度除数下限，单位为 deg/mm。 */
#define ADVANCE_MOTION_PATH_CURVATURE_PREVIEW_MM (300.0f) /*!< 曲率与航向梯度预览距离，单位为 mm。 */
#define ADVANCE_MOTION_PATH_CURVATURE_FF_TIME_S (0.05f) /*!< 曲率法向前馈等效响应时间，单位为秒；设为 0.0f 可关闭。 */

/* 前视与末段捕获。 */
#define ADVANCE_MOTION_PATH_LOOKAHEAD_MIN_MM (60.0f) /*!< 前视下限，单位为 mm。 */
#define ADVANCE_MOTION_PATH_LOOKAHEAD_BASE_MM (60.0f) /*!< 前视基础距离，单位为 mm。 */
#define ADVANCE_MOTION_PATH_LOOKAHEAD_SPEED_GAIN_S (0.15f) /*!< 前视速度增益，单位为 s。 */
#define ADVANCE_MOTION_PATH_LOOKAHEAD_CURVE_GAIN_MM (120.0f) /*!< 前视曲率增益，单位为 mm。 */
#define ADVANCE_MOTION_PATH_LOOKAHEAD_MAX_MM (180.0f) /*!< 前视上限，单位为 mm。 */
#define ADVANCE_MOTION_PATH_LOOKAHEAD_RATE_MM_S (400.0f) /*!< 前视变化率，单位为 mm/s。 */
#define ADVANCE_MOTION_PATH_INITIAL_LOOKAHEAD_MM (80.0f) /*!< 初始前视距离，单位为 mm。 */
#define ADVANCE_MOTION_PATH_FINAL_CAPTURE_DISTANCE_MM (60.0f) /*!< 进入末段捕获距离，单位为 mm。 */
#define ADVANCE_MOTION_PATH_FINAL_CAPTURE_SPEED_MM_S (150.0f) /*!< 进入末段捕获速度，单位为 mm/s。 */

/* 路径横向与航向 PD 默认参数。 */
#define ADVANCE_MOTION_PATH_KP_POS (0.98f) /*!< 路径位置 PD 比例增益，需要单独实机标定。 */
#define ADVANCE_MOTION_PATH_KD_VEL (0.620f) /*!< 路径位置 PD 速度增益，需要单独实机标定。 */
#define ADVANCE_MOTION_PATH_KP_YAW (1.42f) /*!< 路径航向 PD 比例增益，需要单独实机标定。 */
#define ADVANCE_MOTION_PATH_KD_YAW (0.427f) /*!< 路径航向 PD 速度增益，需要单独实机标定。 */

/* 路径超时与底盘驱动。 */
#define ADVANCE_MOTION_PATH_TIMEOUT_BASE_MS ((uint32_t)3000U) /*!< 路径超时基础值，单位为 ms。 */
#define ADVANCE_MOTION_PATH_TIMEOUT_EXPECTED_MIN_SPEED_MM_S (250.0f) /*!< 估算超时最小速度，单位为 mm/s。 */
#define ADVANCE_MOTION_PATH_TIMEOUT_SCALE (2.0f) /*!< 路径超时安全系数，无单位。 */
#define ADVANCE_MOTION_PATH_TIMEOUT_MAX_MS ((uint32_t)60000U) /*!< 路径超时上限，单位为 ms。 */
#define ADVANCE_MOTION_PATH_DRIVER_ACC CHASSIS_DEFAULT_ACC /*!< 驱动协议加速度档位，非软件速度规划加速度。 */

/* 控制周期与传感器数据新鲜度。 */
#define ADVANCE_MOTION_CONTROL_PERIOD_MS ((uint32_t)20U) /*!< 闭环控制周期，单位为 ms。 */
#define ADVANCE_MOTION_POSE_TIMEOUT_MS ((uint32_t)100U) /*!< 位姿数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_YAW_TIMEOUT_MS ((uint32_t)100U) /*!< 航向角数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_ARRIVE_HOLD_MS ((uint32_t)150U) /*!< 到达判定保持时间，单位为 ms。 */

/* PID 公共限制。 */
#define ADVANCE_MOTION_PID_MAX_DT_MS ((uint32_t)100U) /*!< PID 历史允许的最大间隔，单位为 ms。 */

/* 组合 GOTO 航向策略：复位后恢复本编译期默认值。 */
#define ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE ((uint8_t)0U) /*!< 复位后的默认策略：0 为始终并行，1 为大角度先对准。 */
#define ADVANCE_MOTION_LARGE_YAW_ALIGN_ENTER_DEG (30.0f) /*!< 航向绝对误差达到该值时暂停平移并进入对准阶段，单位为度。 */
#define ADVANCE_MOTION_LARGE_YAW_ALIGN_EXIT_DEG (20.0f) /*!< 对准阶段航向绝对误差降至该值后恢复组合平移，单位为度。 */
#define ADVANCE_MOTION_LARGE_YAW_ALIGN_LINEAR_MIN_SCALE (0.35f) /*!< 组合运行时、大航向误差下允许的最小线速度比例。 */

/* 位置 PID 默认参数与在线调参上限。 */
#define ADVANCE_MOTION_DEFAULT_KP_POS (1.5f) /*!< 位置误差比例默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KI_POS (0.10f) /*!< 位置误差积分默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KD_POS (0.78f) /*!< 基于实测速度的位置微分默认增益。 */
#define ADVANCE_MOTION_MAX_KP_POS (20.0f) /*!< 位置误差比例增益上限。 */
#define ADVANCE_MOTION_MAX_KI_POS (20.0f) /*!< 位置误差积分增益上限。 */
#define ADVANCE_MOTION_MAX_KD_POS (20.0f) /*!< 基于实测速度的位置微分增益上限。 */
#define ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S (1000.0f) /*!< 位置误差积分限幅，单位为 mm*s。 */

/* 航向 PID 默认参数与在线调参上限。 */
#define ADVANCE_MOTION_DEFAULT_KP_YAW (2.50f) /*!< 航向角误差比例默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KI_YAW (1.0f) /*!< 航向角误差积分默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KD_YAW (0.80f) /*!< 基于实测角速度的航向微分默认增益。 */
#define ADVANCE_MOTION_MAX_KP_YAW (20.0f) /*!< 航向角误差比例增益上限。 */
#define ADVANCE_MOTION_MAX_KI_YAW (20.0f) /*!< 航向角误差积分增益上限。 */
#define ADVANCE_MOTION_MAX_KD_YAW (20.0f) /*!< 基于实测角速度的航向微分增益上限。 */
#define ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S (180.0f) /*!< 航向角误差积分限幅，单位为 deg*s。 */

/* 到达判定与无进展保护。 */
#define ADVANCE_MOTION_POS_TOLERANCE_MM (10.0f) /*!< 位置到达容差，单位为 mm。 */
#define ADVANCE_MOTION_YAW_TOLERANCE_DEG (1.5f) /*!< 航向角到达容差，单位为度。 */
#define ADVANCE_MOTION_NO_PROGRESS_WINDOW_MS ((uint32_t)2500U) /*!< 无进展判定观察窗口，单位为 ms。 */
#define ADVANCE_MOTION_NO_PROGRESS_MIN_REDUCTION_MM (2.0f) /*!< 观察窗口内要求的最小误差下降量，单位为 mm。 */
#define ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S (30.0f) /*!< 启用无进展判定的最小线速度指令，单位为 mm/s。 */

/* 简化到点接口的默认目标参数。 */
#define ADVANCE_MOTION_DEFAULT_VMAX_MM_S (820.0f) /*!< 默认最大线速度，单位为 mm/s。 */
#define ADVANCE_MOTION_DEFAULT_WMAX_DEG_S (100.0f) /*!< 默认最大角速度，单位为度/s。 */
#define ADVANCE_MOTION_DEFAULT_TIMEOUT_MS ((uint32_t)10000U) /*!< 默认目标超时时间，单位为 ms。 */

/*
 * GotoPose 场地边界。它们是软件安全限值，不替代现场的机械限位。
 * 修改前应确认场地尺寸、OPS 坐标单位和底盘的可制动距离。
 */
#define ADVANCE_MOTION_WORLD_X_MIN_MM (-5000.0f) /*!< 世界坐标 X 最小边界，单位为 mm。 */
#define ADVANCE_MOTION_WORLD_X_MAX_MM (5000.0f) /*!< 世界坐标 X 最大边界，单位为 mm。 */
#define ADVANCE_MOTION_WORLD_Y_MIN_MM (-5000.0f) /*!< 世界坐标 Y 最小边界，单位为 mm。 */
#define ADVANCE_MOTION_WORLD_Y_MAX_MM (5000.0f) /*!< 世界坐标 Y 最大边界，单位为 mm。 */

/* 目标速度与超时时间的输入上限。 */
#define ADVANCE_MOTION_MAX_VMAX_MM_S (1500.0f) /*!< 允许的最大线速度，单位为 mm/s。 */
#define ADVANCE_MOTION_MAX_WMAX_DEG_S (180.0f) /*!< 允许的最大角速度，单位为度/s。 */
#define ADVANCE_MOTION_MAX_TIMEOUT_MS ((uint32_t)60000U) /*!< 允许的最大目标超时时间，单位为 ms。 */

/* 目标可选约束标志。 */
#define ADVANCE_MOTION_GOAL_USE_YAW ((uint8_t)0x01U) /*!< 目标标志：使用航向角约束。 */
#define ADVANCE_MOTION_GOAL_USE_POSITION ((uint8_t)0x02U) /*!< 目标标志：使用 X/Y 位置约束。 */

  typedef enum
  {
    ADVANCE_MOTION_STATUS_OK = 0,
    ADVANCE_MOTION_STATUS_INVALID_PARAM,
    ADVANCE_MOTION_STATUS_NO_ORIGIN,
    ADVANCE_MOTION_STATUS_NO_POSE,
    ADVANCE_MOTION_STATUS_POSE_TIMEOUT,
    ADVANCE_MOTION_STATUS_BUSY
  } AdvanceMotion_Status_t;

  typedef enum
  {
    ADVANCE_MOTION_STATE_IDLE = 0,
    ADVANCE_MOTION_STATE_RUNNING,
    ADVANCE_MOTION_STATE_ARRIVED,
    ADVANCE_MOTION_STATE_TIMEOUT,
    ADVANCE_MOTION_STATE_NO_POSE,
    ADVANCE_MOTION_STATE_NO_ORIGIN,
    ADVANCE_MOTION_STATE_CANCELED
  } AdvanceMotion_RunState_t;

  /** @brief 位姿外环 PID 的完整运行时参数组。 */
  typedef struct
  {
    float kp_pos; /*!< X/Y 位置误差比例增益。 */
    float ki_pos; /*!< X/Y 位置误差积分增益。 */
    float kd_pos; /*!< X/Y 基于实测速度的微分增益。 */
    float kp_yaw; /*!< 航向角误差比例增益。 */
    float ki_yaw; /*!< 航向角误差积分增益。 */
    float kd_yaw; /*!< 航向角基于实测角速度的微分增益。 */
  } AdvanceMotion_PidConfig_t;

  /** @brief 连续路径控制器与速度规划器的完整运行时参数组。 */
  typedef struct
  {
    /* 横向与航向反馈。 */
    float kp_cross_track;
    float kd_cross_track_velocity;
    float kp_yaw;
    float kd_yaw_rate;

    /* 速度与加速度约束。 */
    float cruise_speed_mm_s;
    float max_yaw_rate_deg_s;
    float accel_mm_s2;
    float decel_mm_s2;
    float max_lateral_accel_mm_s2;

    /* 曲率预览与法向前馈。 */
    float curvature_preview_mm;
    float curvature_ff_time_s;

    /* 前视距离规划。 */
    float lookahead_min_mm;
    float lookahead_base_mm;
    float lookahead_speed_gain_s;
    float lookahead_curve_gain_mm;
    float lookahead_max_mm;
    float lookahead_rate_mm_s;
    float initial_lookahead_mm;

    /* 末段捕获。 */
    float final_capture_distance_mm;
    float final_capture_speed_mm_s;
  } AdvanceMotion_PathControlConfig_t;

  typedef struct
  {
    float x_mm;
    float y_mm;
    float yaw_deg;
  } AdvanceMotion_PathPoint_t;

  /** @brief 运动控制的对外状态快照，用于上位机与调试查询。 */
  typedef struct
  {
    AdvanceMotion_RunState_t state; /*!< 当前运行状态。 */
    WorldGoalPose2D_t goal; /*!< 当前目标位姿。 */
    WorldPose2D_t pose; /*!< 当前实际位姿。 */
    float error_x_mm; /*!< X 方向位置误差，单位为 mm。 */
    float error_y_mm; /*!< Y 方向位置误差，单位为 mm。 */
    float position_error_mm; /*!< 合成位置误差，单位为 mm。 */
    float yaw_error_deg; /*!< 航向角误差，单位为度。 */
    uint32_t started_tick; /*!< 任务开始时间，单位为 ms。 */
    uint32_t updated_tick; /*!< 状态更新时间，单位为 ms。 */
  } AdvanceMotion_RuntimeStatus_t;

  /* Debug snapshot refreshed once per motion-control period. */
  typedef struct
  {
    uint32_t tick;
    uint32_t pid_revision;
    uint32_t path_config_revision;
    AdvanceMotion_RunState_t state;
    uint8_t flags;
    WorldGoalPose2D_t goal;
    WorldPose2D_t pose;
    float error_x_mm;
    float error_y_mm;
    float error_yaw_deg;
    float command_vx_world_mm_s;
    float command_vy_world_mm_s;
    float command_wz_ccw_deg_s;
    float measured_vx_world_mm_s;
    float measured_vy_world_mm_s;
    float measured_wz_deg_s;
    float integral_x_mm_s;
    float integral_y_mm_s;
    float integral_yaw_deg_s;
    uint16_t nearest_segment_index; /*!< 当前投影点所在的路径段索引。 */
    uint16_t target_segment_index; /*!< 动态前视点所在的路径段索引。 */
    float path_progress_mm; /*!< 从路径起点到投影点的累计弧长，单位为 mm。 */
    float path_remaining_mm; /*!< 从投影点到最终点的剩余弧长，单位为 mm。 */
    float path_projection_x_mm; /*!< 当前路径投影点 world X，单位为 mm。 */
    float path_projection_y_mm; /*!< 当前路径投影点 world Y，单位为 mm。 */
    float path_lookahead_x_mm; /*!< 动态前视点 world X，单位为 mm。 */
    float path_lookahead_y_mm; /*!< 动态前视点 world Y，单位为 mm。 */
    float path_signed_curvature_1_mm; /*!< 当前局部带符号曲率，单位为 1/mm。 */
    float path_curvature_preview_1_mm; /*!< 预览窗口内的最大绝对曲率，单位为 1/mm。 */
    float path_yaw_gradient_deg_per_mm; /*!< 投影段的有符号航向梯度，单位为 deg/mm。 */
    float path_reference_speed_mm_s; /*!< 加减速约束后的路径参考速度，单位为 mm/s。 */
    float path_lookahead_mm; /*!< 当前动态前视距离，单位为 mm。 */
    float path_feedforward_vx_mm_s; /*!< 路径切向 X 速度前馈，单位为 mm/s。 */
    float path_feedforward_vy_mm_s; /*!< 路径切向 Y 速度前馈，单位为 mm/s。 */
    float path_feedforward_wz_deg_s; /*!< 路径航向变化率前馈，单位为 deg/s。 */
    float path_cross_track_mm; /*!< 当前横向误差，单位为 mm。 */
    float path_measured_normal_velocity_mm_s; /*!< 实测左法向速度，单位为 mm/s。 */
    float path_normal_velocity_ff_mm_s; /*!< 曲率法向速度前馈，单位为 mm/s。 */
    float path_normal_feedback_mm_s; /*!< 横向 PD 合成修正量，单位为 mm/s。 */
    float path_command_wz_deg_s; /*!< 路径分支最终角速度指令，单位为 deg/s。 */
    uint8_t path_final_stage; /*!< 非零表示已进入最终 Goto PID 捕获阶段。 */
  } AdvanceMotion_DebugSnapshot_t;

/* POSE_FRESH 与 YAW_FRESH 分别反映 OPS 位置和当前航向源的真实时间戳新鲜度。 */
#define ADVANCE_MOTION_DEBUG_FLAG_VALID ((uint8_t)0x01U)
#define ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH ((uint8_t)0x02U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH ((uint8_t)0x04U)
#define ADVANCE_MOTION_DEBUG_FLAG_LINEAR_SATURATED ((uint8_t)0x08U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_SATURATED ((uint8_t)0x10U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_ALIGNING ((uint8_t)0x20U)
#define ADVANCE_MOTION_DEBUG_FLAG_PATH_ACTIVE ((uint8_t)0x40U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_SOURCE_OPS ((uint8_t)0x80U)

  /** @brief 初始化运动控制模块。 */
  void AdvanceMotion_Init(void);
  /** @brief 设置世界坐标系速度及加速度。 @return 设置结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc);
  /** @brief 启动带加速度参数的位姿导航。 @param goal 目标位姿指针。 @return 启动结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc);
  /**
   * @brief 异步启动连续路径跟踪。
   * @details points 必须指向调用方长期持有的只读数组；任务结束、取消或进入异常终态前，
   * 调用方不得释放、覆盖或修改该数组。中间点是连续通过的软途经点，最后一个点是精确收敛并停车的终点。
   * @param points 路径离散采样点数组。
   * @param point_count 路径点数量，至少为 2。
   * @return 启动结果状态。
   */
  AdvanceMotion_Status_t AdvanceMotion_FollowPathEx(const AdvanceMotion_PathPoint_t *points,
                                                     uint16_t point_count);
  /** @brief 阻塞执行连续路径；等待期间仅执行 __WFI()，不得从中断上下文调用。 */
  AdvanceMotion_RunState_t AdvanceMotion_FollowPathBlocking(const AdvanceMotion_PathPoint_t *points,
                                                             uint16_t point_count);
  /**
   * @brief 阻塞执行位姿导航，返回前不会继续执行调用方后续代码。
   * @details UART/DMA 中断在阻塞期间仍可运行，但本函数不处理上位机协议队列。
   * 外部通信控制应使用异步 AdvanceMotion_GotoPoseEx；本接口用于本地测试和固定顺序业务流程。
   * @return 最终 AdvanceMotion_RunState_t，不得从中断上下文调用。
   */
  AdvanceMotion_RunState_t AdvanceMotion_GotoGoalBlocking(const WorldGoalPose2D_t *goal, uint8_t acc);
  /** @brief 使用默认速度、超时和航向约束阻塞执行到点运动。*/
  AdvanceMotion_RunState_t AdvanceMotion_GotoPoseBlocking(float x_mm, float y_mm,
                                                           float yaw_deg, uint8_t acc);
  /** @brief 由 TIM6 按控制周期推进一次位姿导航控制器。 */
  void AdvanceMotion_Update(void);
  /** @brief 仅在存在活动目标时取消、释放控制权并停车。 */
  void AdvanceMotion_CancelIfActive(void);
  /** @brief 取消当前导航目标并停车。 */
  void AdvanceMotion_Cancel(void);
  /** @brief 获取运动控制运行状态。 @param status 输出状态结构体。 @return 获取结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status);
  /** @brief Get one consistent PID debug snapshot without performing I/O. */
  AdvanceMotion_Status_t AdvanceMotion_GetDebugSnapshot(AdvanceMotion_DebugSnapshot_t *snapshot);
  /** @brief 获取当前生效的 PID 配置与版本号。 */
  AdvanceMotion_Status_t AdvanceMotion_GetPidConfig(AdvanceMotion_PidConfig_t *config,
                                                      uint32_t *revision);
  /** @brief 校验并提交完整 PID 配置，在下一次 20 ms 周期边界整体生效。 */
  AdvanceMotion_Status_t AdvanceMotion_RequestPidConfig(const AdvanceMotion_PidConfig_t *config,
                                                          uint32_t *revision);
  /** @brief 提交固件默认 PID 配置，在下一次 20 ms 周期边界整体生效。 */
  AdvanceMotion_Status_t AdvanceMotion_RestoreDefaultPid(uint32_t *revision);
  /** @brief 获取当前生效的连续路径参数与版本号。 */
  AdvanceMotion_Status_t AdvanceMotion_GetPathControlConfig(
      AdvanceMotion_PathControlConfig_t *config, uint32_t *revision);
  /** @brief 提交连续路径参数，在下一次 20 ms 控制周期边界整体生效。 */
  AdvanceMotion_Status_t AdvanceMotion_RequestPathControlConfig(
      const AdvanceMotion_PathControlConfig_t *config, uint32_t *revision);
  /** @brief 恢复固件默认连续路径参数。 */
  AdvanceMotion_Status_t AdvanceMotion_RestoreDefaultPathControl(uint32_t *revision);
  /** @brief 设置组合 GOTO 的大航向误差先对准策略；仅空闲状态可修改。 */
  AdvanceMotion_Status_t AdvanceMotion_SetLargeYawAlignEnabled(uint8_t enabled);
  /** @brief 获取组合 GOTO 的大航向误差先对准策略当前状态。 */
  AdvanceMotion_Status_t AdvanceMotion_GetLargeYawAlignEnabled(uint8_t *enabled);
  /** @brief 航向数据源变更后清除航向 PID 历史，避免使用旧源的积分与微分项。 */
  void AdvanceMotion_ResetYawControl(void);

#ifdef __cplusplus
}
#endif

#endif
