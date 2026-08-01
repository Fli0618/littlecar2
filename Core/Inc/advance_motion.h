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

/* 控制周期与传感器数据新鲜度。 */
#define ADVANCE_MOTION_CONTROL_PERIOD_MS ((uint32_t)20U) /*!< 闭环控制周期，单位为 ms。 */
#define ADVANCE_MOTION_POSE_TIMEOUT_MS ((uint32_t)100U) /*!< 位姿数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_YAW_TIMEOUT_MS ((uint32_t)100U) /*!< 航向角数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_ARRIVE_HOLD_MS ((uint32_t)150U) /*!< 到达判定保持时间，单位为 ms。 */

/* PID 公共限制。 */
#define ADVANCE_MOTION_PID_MAX_DT_MS ((uint32_t)100U) /*!< PID 历史允许的最大间隔，单位为 ms。 */

/* 位置 PID 默认参数与在线调参上限。 */
#define ADVANCE_MOTION_DEFAULT_KP_POS (1.0f) /*!< 位置误差比例默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KI_POS (0.03f) /*!< 位置误差积分默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KD_POS (0.1f) /*!< 基于实测速度的位置微分默认增益。 */
#define ADVANCE_MOTION_MAX_KP_POS (20.0f) /*!< 位置误差比例增益上限。 */
#define ADVANCE_MOTION_MAX_KI_POS (20.0f) /*!< 位置误差积分增益上限。 */
#define ADVANCE_MOTION_MAX_KD_POS (20.0f) /*!< 基于实测速度的位置微分增益上限。 */
#define ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S (1000.0f) /*!< 位置误差积分限幅，单位为 mm*s。 */

/* 航向 PID 默认参数与在线调参上限。 */
#define ADVANCE_MOTION_DEFAULT_KP_YAW (1.9f) /*!< 航向角误差比例默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KI_YAW (0.9f) /*!< 航向角误差积分默认增益。 */
#define ADVANCE_MOTION_DEFAULT_KD_YAW (0.2f) /*!< 基于实测角速度的航向微分默认增益。 */
#define ADVANCE_MOTION_MAX_KP_YAW (20.0f) /*!< 航向角误差比例增益上限。 */
#define ADVANCE_MOTION_MAX_KI_YAW (20.0f) /*!< 航向角误差积分增益上限。 */
#define ADVANCE_MOTION_MAX_KD_YAW (20.0f) /*!< 基于实测角速度的航向微分增益上限。 */
#define ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S (180.0f) /*!< 航向角误差积分限幅，单位为 deg*s。 */

/* 到达判定与无进展保护。 */
#define ADVANCE_MOTION_POS_TOLERANCE_MM (8.0f) /*!< 位置到达容差，单位为 mm。 */
#define ADVANCE_MOTION_YAW_TOLERANCE_DEG (1.0f) /*!< 航向角到达容差，单位为度。 */
#define ADVANCE_MOTION_NO_PROGRESS_WINDOW_MS ((uint32_t)2500U) /*!< 无进展判定观察窗口，单位为 ms。 */
#define ADVANCE_MOTION_NO_PROGRESS_MIN_REDUCTION_MM (2.0f) /*!< 观察窗口内要求的最小误差下降量，单位为 mm。 */
#define ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S (30.0f) /*!< 启用无进展判定的最小线速度指令，单位为 mm/s。 */

/* 简化到点接口的默认目标参数。 */
#define ADVANCE_MOTION_DEFAULT_VMAX_MM_S (200.0f) /*!< 默认最大线速度，单位为 mm/s。 */
#define ADVANCE_MOTION_DEFAULT_WMAX_DEG_S (90.0f) /*!< 默认最大角速度，单位为度/s。 */
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
  } AdvanceMotion_DebugSnapshot_t;

#define ADVANCE_MOTION_DEBUG_FLAG_VALID ((uint8_t)0x01U)
#define ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH ((uint8_t)0x02U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH ((uint8_t)0x04U)
#define ADVANCE_MOTION_DEBUG_FLAG_LINEAR_SATURATED ((uint8_t)0x08U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_SATURATED ((uint8_t)0x10U)
#define ADVANCE_MOTION_DEBUG_FLAG_YAW_SOURCE_OPS ((uint8_t)0x80U)

  /** @brief 初始化运动控制模块。 */
  void AdvanceMotion_Init(void);
  /** @brief 设置世界坐标系速度及加速度。 @return 设置结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc);
  /** @brief 启动带加速度参数的位姿导航。 @param goal 目标位姿指针。 @return 启动结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc);
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
  /** @brief 航向数据源变更后清除航向 PID 历史，避免使用旧源的积分与微分项。 */
  void AdvanceMotion_ResetYawControl(void);

#ifdef __cplusplus
}
#endif

#endif
