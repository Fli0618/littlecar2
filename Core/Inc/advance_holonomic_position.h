#ifndef __ADVANCE_HOLONOMIC_POSITION_H__
#define __ADVANCE_HOLONOMIC_POSITION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "advance_world.h"

/*
 * 目标约束标志位，数值与 ADVANCE_MOTION_GOAL_USE_* 一致，
 * 可直接接受调用方按现有约定传入的 WorldGoalPose2D_t.goal_flags。
 */
#define ADVANCE_HOLONOMIC_GOAL_USE_YAW ((uint8_t)0x01U)       /* 使用航向约束 */
#define ADVANCE_HOLONOMIC_GOAL_USE_POSITION ((uint8_t)0x02U)  /* 使用 X/Y 位置约束 */

/** @brief 全向位置控制器的接口返回状态。 */
typedef enum
{
  ADVANCE_HOLONOMIC_STATUS_OK = 0,
  ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM,
  ADVANCE_HOLONOMIC_STATUS_NO_ORIGIN,
  ADVANCE_HOLONOMIC_STATUS_NO_POSE,
  ADVANCE_HOLONOMIC_STATUS_POSE_TIMEOUT,
  ADVANCE_HOLONOMIC_STATUS_BUSY
} AdvanceHolonomic_Status_t;

/** @brief 全向位置控制器的运行状态。 */
typedef enum
{
  ADVANCE_HOLONOMIC_STATE_IDLE = 0,
  ADVANCE_HOLONOMIC_STATE_RUNNING,
  ADVANCE_HOLONOMIC_STATE_SETTLING,
  ADVANCE_HOLONOMIC_STATE_ARRIVED,
  ADVANCE_HOLONOMIC_STATE_TIMEOUT,
  ADVANCE_HOLONOMIC_STATE_NO_POSE,
  ADVANCE_HOLONOMIC_STATE_NO_ORIGIN,
  ADVANCE_HOLONOMIC_STATE_CANCELED
} AdvanceHolonomic_RunState_t;

/**
 * @brief 全向位置控制器运行时可调参数，共 12 个字段。
 * @note 单位约定：位置 mm、速度 mm/s、角速度 deg/s、角度 deg。
 */
typedef struct
{
  float linear_accel_mm_s2;  /*!< 平移运动轮廓加速度，单位 mm/s^2 */
  float linear_decel_mm_s2;  /*!< 平移运动轮廓减速度，单位 mm/s^2 */
  float yaw_accel_deg_s2;    /*!< 航向运动轮廓角加速度（对称加减速），单位 deg/s^2 */
  float kp_forward;          /*!< 车体前向位置反馈增益，单位 1/s */
  float kv_forward;          /*!< 车体前向速度反馈增益，无量纲 */
  float kp_lateral;          /*!< 车体横向位置反馈增益，单位 1/s */
  float kv_lateral;          /*!< 车体横向速度反馈增益，无量纲 */
  float kp_yaw;              /*!< 航向位置反馈增益，单位 1/s */
  float kv_yaw;              /*!< 航向速度反馈增益，无量纲 */
  float forward_scale;       /*!< 前向驱动比例校准，无量纲 */
  float lateral_scale;       /*!< 横向驱动比例校准，无量纲 */
  float yaw_scale;           /*!< 旋转驱动比例校准，无量纲 */
} AdvanceHolonomic_Config_t;

/** @brief 全向位置控制器的对外状态摘要。 */
typedef struct
{
  AdvanceHolonomic_RunState_t state; /*!< 当前运行状态 */
  WorldGoalPose2D_t goal;            /*!< 当前目标位姿 */
  WorldPose2D_t actual_pose;         /*!< 当前实际位姿 */
  float position_error_mm;           /*!< 当前位置误差，单位 mm；位置轴未启用时为 0 */
  float yaw_error_deg;               /*!< 当前航向误差，单位 deg；航向轴未启用时为 0 */
  uint32_t started_tick;             /*!< 任务开始时间，单位 ms */
  uint32_t updated_tick;             /*!< 状态更新时间，单位 ms */
} AdvanceHolonomic_RuntimeStatus_t;

/** @brief 全向位置控制器调试快照，仅内存更新，不在控制周期打印。 */
typedef struct
{
  uint32_t tick;                        /*!< 快照时间，单位 ms */
  AdvanceHolonomic_RunState_t state;    /*!< 当前运行状态 */
  WorldGoalPose2D_t goal;               /*!< 当前目标位姿 */
  WorldPose2D_t actual_pose;            /*!< 当前实际位姿 */
  float reference_x_mm;                 /*!< 参考位置 X，单位 mm */
  float reference_y_mm;                 /*!< 参考位置 Y，单位 mm */
  float reference_yaw_deg;              /*!< 参考航向，单位 deg */
  float reference_vx_world_mm_s;        /*!< 参考世界速度 X，单位 mm/s */
  float reference_vy_world_mm_s;        /*!< 参考世界速度 Y，单位 mm/s */
  float reference_wz_deg_s;             /*!< 参考角速度，单位 deg/s */
  float error_forward_mm;               /*!< 车体前向位置误差，单位 mm */
  float error_lateral_mm;               /*!< 车体横向位置误差，单位 mm */
  float error_yaw_deg;                  /*!< 航向误差，单位 deg */
  float measured_forward_mm_s;          /*!< 实测车体前向速度，单位 mm/s */
  float measured_lateral_mm_s;          /*!< 实测车体横向速度，单位 mm/s */
  float measured_wz_deg_s;              /*!< 实测角速度，单位 deg/s */
  float position_correction_forward_mm_s; /*!< 前向位置修正量，单位 mm/s */
  float position_correction_lateral_mm_s; /*!< 横向位置修正量，单位 mm/s */
  float velocity_correction_forward_mm_s; /*!< 前向速度修正量，单位 mm/s */
  float velocity_correction_lateral_mm_s; /*!< 横向速度修正量，单位 mm/s */
  float yaw_position_correction_deg_s;    /*!< 航向位置修正量，单位 deg/s */
  float yaw_velocity_correction_deg_s;    /*!< 航向速度修正量，单位 deg/s */
  float command_forward_mm_s;           /*!< 车体前向速度命令，单位 mm/s */
  float command_lateral_mm_s;           /*!< 车体横向速度命令，单位 mm/s */
  float command_wz_deg_s;               /*!< 航向速度命令，单位 deg/s */
  float drive_forward_mm_s;             /*!< 校准后前向速度，单位 mm/s */
  float drive_lateral_mm_s;             /*!< 校准后横向速度，单位 mm/s */
  float drive_wz_deg_s;                 /*!< 校准后角速度，单位 deg/s */
  float profile_progress_mm;            /*!< 平移轮廓进度，单位 mm */
  float profile_remaining_mm;           /*!< 平移轮廓剩余距离，单位 mm */
  float profile_reference_speed_mm_s;   /*!< 平移轮廓参考速度，单位 mm/s */
} AdvanceHolonomic_DebugSnapshot_t;

/** @brief 初始化全向位置控制器。 */
void AdvanceHolonomic_Init(void);

/**
 * @brief 启动全向位置任务。
 * @param goal 目标位姿指针，复用 WorldGoalPose2D_t。
 * @param driver_acc 底盘驱动加速度档位，原样传给 Chassis_SetBodyVelocityEx。
 * @return 启动结果状态。
 */
AdvanceHolonomic_Status_t AdvanceHolonomic_Start(
    const WorldGoalPose2D_t *goal,
    uint8_t driver_acc);

/** @brief 阻塞执行目标位姿任务，返回最终运行状态；仅 Start + __WFI() 薄封装。 */
AdvanceHolonomic_RunState_t AdvanceHolonomic_GotoGoalBlocking(
    const WorldGoalPose2D_t *goal,
    uint8_t driver_acc);

/** @brief 阻塞执行到点任务（使用默认速度、超时和位置+航向约束）。 */
AdvanceHolonomic_RunState_t AdvanceHolonomic_GotoPoseBlocking(
    float x_mm,
    float y_mm,
    float yaw_deg,
    uint8_t driver_acc);

/** @brief 由 TIM6 20 ms 控制周期调用，推进轮廓、反馈控制与状态机。 */
void AdvanceHolonomic_Update(void);

/** @brief 主动取消当前任务：先平滑停车再释放控制权。 */
void AdvanceHolonomic_Cancel(void);

/** @brief 查询控制器是否处于活动状态（RUNNING 或 SETTLING）。 */
uint8_t AdvanceHolonomic_IsActive(void);

/** @brief 获取对外状态摘要。 */
AdvanceHolonomic_Status_t AdvanceHolonomic_GetStatus(
    AdvanceHolonomic_RuntimeStatus_t *status);

/** @brief 获取调试快照（临界区整体拷贝）。 */
AdvanceHolonomic_Status_t AdvanceHolonomic_GetDebugSnapshot(
    AdvanceHolonomic_DebugSnapshot_t *snapshot);

/**
 * @brief 获取当前生效的运行时参数与活动修订号。
 * @param config 输出 12 项参数。
 * @param revision 输出当前 active 修订号。
 */
AdvanceHolonomic_Status_t AdvanceHolonomic_GetConfig(
    AdvanceHolonomic_Config_t *config,
    uint32_t *revision);

/**
 * @brief 校验并整组提交运行时参数，在下一次 20 ms 周期边界原子生效。
 * @param config 输入 12 项参数。
 * @param revision 输出本次提交的 pending 修订号。
 * @note 运行期间允许提交；Kp/Kv/scale 下一控制周期生效，
 *       已预计算的 accel/decel 轮廓不重建，下一次 Start 才使用新值。
 */
AdvanceHolonomic_Status_t AdvanceHolonomic_RequestConfig(
    const AdvanceHolonomic_Config_t *config,
    uint32_t *revision);

/** @brief 恢复保守默认参数，同样走 pending/revision 流程。 */
AdvanceHolonomic_Status_t AdvanceHolonomic_RestoreDefaultConfig(
    uint32_t *revision);

#ifdef __cplusplus
}
#endif

#endif
