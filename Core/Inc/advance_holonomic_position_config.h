#ifndef __ADVANCE_HOLONOMIC_POSITION_CONFIG_H__
#define __ADVANCE_HOLONOMIC_POSITION_CONFIG_H__

#include <stdint.h>

/*
 * 全向位置控制器固定安全参数与默认运行时参数。
 * 固定安全参数集中在此文件，不参与运行时配置；
 * 运行时可调参数仅由 AdvanceHolonomic_Config_t 的 12 个字段构成。
 */

/* 数据新鲜度与时间约束 */
#define ADVANCE_HOLONOMIC_POSE_TIMEOUT_MS ((uint32_t)100U)  /* OPS 位置数据超时阈值，单位 ms */
#define ADVANCE_HOLONOMIC_YAW_TIMEOUT_MS ((uint32_t)100U)   /* 当前航向源数据超时阈值，单位 ms */
#define ADVANCE_HOLONOMIC_MAX_DT_MS ((uint32_t)100U)        /* 轮廓/速度估计允许的最大时间间隔，单位 ms */

/* 速度估计：一阶低通系数，越小越平滑；固定宏，不做运行时调参 */
#define ADVANCE_HOLONOMIC_VEL_FILTER_ALPHA (0.2f)

/* 到达判定（未启用的轴不检查） */
#define ADVANCE_HOLONOMIC_POSITION_TOLERANCE_MM (10.0f)        /* 位置误差阈值，单位 mm */
#define ADVANCE_HOLONOMIC_LINEAR_SPEED_TOLERANCE_MM_S (40.0f)  /* 实际线速度阈值，单位 mm/s */
#define ADVANCE_HOLONOMIC_YAW_TOLERANCE_DEG (1.5f)             /* 航向误差阈值，单位 deg */
#define ADVANCE_HOLONOMIC_YAW_RATE_TOLERANCE_DEG_S (4.0f)      /* 实际角速度阈值，单位 deg/s */
#define ADVANCE_HOLONOMIC_ARRIVE_HOLD_MS ((uint32_t)150U)      /* 到达条件连续保持时间，单位 ms */

/* 反馈合成修正量限幅（固定安全宏，防止反馈无限放大） */
#define ADVANCE_HOLONOMIC_MAX_FORWARD_CORRECTION_MM_S (400.0f) /* 前向修正上限，单位 mm/s */
#define ADVANCE_HOLONOMIC_MAX_LATERAL_CORRECTION_MM_S (400.0f) /* 横向修正上限，单位 mm/s */
#define ADVANCE_HOLONOMIC_MAX_YAW_CORRECTION_DEG_S (60.0f)     /* 航向修正上限，单位 deg/s */

/* 平移标量路径：小于该长度视为零距离目标 */
#define ADVANCE_HOLONOMIC_MIN_PATH_LENGTH_MM (1.0f)

/* 目标校验边界（数值镜像现有 AdvanceMotion 安全边界，模块不依赖其头文件） */
#define ADVANCE_HOLONOMIC_WORLD_X_MIN_MM (-5000.0f)
#define ADVANCE_HOLONOMIC_WORLD_X_MAX_MM (5000.0f)
#define ADVANCE_HOLONOMIC_WORLD_Y_MIN_MM (-5000.0f)
#define ADVANCE_HOLONOMIC_WORLD_Y_MAX_MM (5000.0f)
#define ADVANCE_HOLONOMIC_MAX_VMAX_MM_S (1500.0f)      /* 允许的最大线速度，单位 mm/s */
#define ADVANCE_HOLONOMIC_MAX_WMAX_DEG_S (180.0f)      /* 允许的最大角速度，单位 deg/s */
#define ADVANCE_HOLONOMIC_MAX_TIMEOUT_MS ((uint32_t)60000U) /* 允许的最大目标超时，单位 ms */

/* 简化到点接口的默认目标参数 */
#define ADVANCE_HOLONOMIC_DEFAULT_VMAX_MM_S (820.0f)   /* 默认最大线速度，单位 mm/s */
#define ADVANCE_HOLONOMIC_DEFAULT_WMAX_DEG_S (100.0f)  /* 默认最大角速度，单位 deg/s */
#define ADVANCE_HOLONOMIC_DEFAULT_TIMEOUT_MS ((uint32_t)10000U) /* 默认目标超时，单位 ms */

/* 默认运行时参数（保守初值，均需实车标定，不作为已标定参数） */
#define ADVANCE_HOLONOMIC_DEFAULT_LINEAR_ACCEL_MM_S2 (600.0f) /* 平移加速度，单位 mm/s^2 */
#define ADVANCE_HOLONOMIC_DEFAULT_LINEAR_DECEL_MM_S2 (800.0f) /* 平移减速度，单位 mm/s^2 */
#define ADVANCE_HOLONOMIC_DEFAULT_YAW_ACCEL_DEG_S2 (150.0f)   /* 航向角加速度（对称），单位 deg/s^2 */
#define ADVANCE_HOLONOMIC_DEFAULT_KP_FORWARD (0.8f)           /* 前向位置增益，单位 1/s */
#define ADVANCE_HOLONOMIC_DEFAULT_KV_FORWARD (0.3f)           /* 前向速度增益，无量纲 */
#define ADVANCE_HOLONOMIC_DEFAULT_KP_LATERAL (0.8f)           /* 横向位置增益，单位 1/s */
#define ADVANCE_HOLONOMIC_DEFAULT_KV_LATERAL (0.3f)           /* 横向速度增益，无量纲 */
#define ADVANCE_HOLONOMIC_DEFAULT_KP_YAW (2.0f)               /* 航向位置增益，单位 1/s */
#define ADVANCE_HOLONOMIC_DEFAULT_KV_YAW (0.3f)               /* 航向速度增益，无量纲 */
#define ADVANCE_HOLONOMIC_DEFAULT_FORWARD_SCALE (1.0f)        /* 前向驱动比例校准，无量纲 */
#define ADVANCE_HOLONOMIC_DEFAULT_LATERAL_SCALE (1.0f)        /* 横向驱动比例校准，无量纲 */
#define ADVANCE_HOLONOMIC_DEFAULT_YAW_SCALE (1.0f)            /* 旋转驱动比例校准，无量纲 */

/* 配置校验边界 */
#define ADVANCE_HOLONOMIC_MAX_GAIN (20.0f)   /* 增益上限，单位 1/s 或无量纲 */
#define ADVANCE_HOLONOMIC_MIN_SCALE (0.5f)   /* scale 下限，无量纲 */
#define ADVANCE_HOLONOMIC_MAX_SCALE (2.0f)   /* scale 上限，无量纲 */
#define ADVANCE_HOLONOMIC_MAX_ACCEL_MM_S2 (5000.0f)   /* 平移加速度上限，单位 mm/s^2 */
#define ADVANCE_HOLONOMIC_MAX_YAW_ACCEL_DEG_S2 (5000.0f) /* 航向角加速度上限，单位 deg/s^2 */

#endif
