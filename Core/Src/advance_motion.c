#include "advance_motion.h"
#include "advance_motion_config.h"

typedef struct
{
  WorldGoalPose2D_t goal;
  WorldPose2D_t pose;
  float error_x_mm;
  float error_y_mm;
  float position_error_mm;
  float yaw_error_deg;
  uint32_t started_tick;
  uint32_t updated_tick;
} AdvanceMotion_RuntimeData_t;

typedef struct
{
  float signed_curvature_1_mm;
  float absolute_curvature_1_mm;
} AdvanceMotion_PathVertexCurvature_t;

typedef struct
{
  uint32_t arrive_hold_start_tick;
  uint32_t pid_last_tick;
  uint32_t no_progress_start_tick;
  uint32_t last_pose_updated_tick;
  uint32_t last_yaw_updated_tick;
  float pid_integral_x_mm_s;
  float pid_integral_y_mm_s;
  float pid_integral_yaw_deg_s;
  float pid_last_x_mm;
  float pid_last_y_mm;
  float pid_last_yaw_deg;
  float measured_vx_world_mm_s;
  float measured_vy_world_mm_s;
  float measured_wz_deg_s;
  float command_vx_world_mm_s;
  float command_vy_world_mm_s;
  float command_wz_ccw_deg_s;
  float no_progress_reference_error_mm;
  uint8_t arrival_stop_sent;
  uint8_t arrival_hard_stop_sent;
  uint8_t terminal_stop_pending;
  uint8_t pid_history_valid;
  uint8_t large_yaw_align_enabled;
  uint8_t yaw_aligning;
  uint8_t acc;
} AdvanceMotion_Control_t;

typedef struct
{
  const AdvanceMotion_PathPoint_t *points;
  uint16_t point_count;
  uint16_t nearest_index;
  uint16_t target_index;
  uint32_t progress_tick;
  uint32_t reference_tick;
  uint32_t timeout_ms;
  float progress_on_segment;
  float completed_length_mm;
  float total_length_mm;
  float progress_mm;
  float progress_reference_mm;
  float remaining_mm;
  float projection_x_mm;
  float projection_y_mm;
  float cross_track_mm;
  float lookahead_mm;
  float signed_curvature_1_mm;
  float curvature_preview_1_mm;
  float yaw_gradient_deg_per_mm;
  float yaw_gradient_preview_deg_per_mm;
  float reference_speed_mm_s;
  float feedforward_vx_mm_s;
  float feedforward_vy_mm_s;
  float feedforward_wz_deg_s;
  float measured_normal_velocity_mm_s;
  float normal_velocity_ff_mm_s;
  float normal_feedback_mm_s;
  float command_wz_deg_s;
  uint8_t final_stage;
  uint8_t active;
} AdvanceMotion_PathContext_t;

static AdvanceMotion_RuntimeData_t g_motion = {0};
static AdvanceMotion_DebugSnapshot_t g_motion_debug = {0};
static AdvanceMotion_Control_t g_motion_control = {0};
static AdvanceMotion_PathContext_t g_path = {0};
static volatile AdvanceMotion_RunState_t g_motion_state = ADVANCE_MOTION_STATE_IDLE;
static AdvanceMotion_PidConfig_t g_pid_active;
static AdvanceMotion_PidConfig_t g_pid_pending;
static volatile uint8_t g_pid_pending_valid;
static volatile uint32_t g_pid_active_revision;
static volatile uint32_t g_pid_pending_revision;
static volatile uint32_t g_pid_next_revision;
static AdvanceMotion_PathControlConfig_t g_path_config_active;
static AdvanceMotion_PathControlConfig_t g_path_config_pending;
static volatile uint8_t g_path_config_pending_valid;
static volatile uint32_t g_path_config_active_revision;
static volatile uint32_t g_path_config_pending_revision;
static volatile uint32_t g_path_config_next_revision;
static uint8_t g_large_yaw_align_enabled;

static const AdvanceMotion_PidConfig_t g_pid_default = {
    .kp_pos = ADVANCE_MOTION_DEFAULT_KP_POS,
    .ki_pos = ADVANCE_MOTION_DEFAULT_KI_POS,
    .kd_pos = ADVANCE_MOTION_DEFAULT_KD_POS,
    .kp_yaw = ADVANCE_MOTION_DEFAULT_KP_YAW,
    .ki_yaw = ADVANCE_MOTION_DEFAULT_KI_YAW,
    .kd_yaw = ADVANCE_MOTION_DEFAULT_KD_YAW};

static const AdvanceMotion_PathControlConfig_t g_path_config_default = {
    .kp_cross_track = ADVANCE_MOTION_PATH_KP_POS,
    .kd_cross_track_velocity = ADVANCE_MOTION_PATH_KD_VEL,
    .kp_yaw = ADVANCE_MOTION_PATH_KP_YAW,
    .kd_yaw_rate = ADVANCE_MOTION_PATH_KD_YAW,
    .cruise_speed_mm_s = ADVANCE_MOTION_PATH_CRUISE_SPEED_MM_S,
    .max_yaw_rate_deg_s = ADVANCE_MOTION_PATH_MAX_WZ_DEG_S,
    .accel_mm_s2 = ADVANCE_MOTION_PATH_ACCEL_MM_S2,
    .decel_mm_s2 = ADVANCE_MOTION_PATH_DECEL_MM_S2,
    .max_lateral_accel_mm_s2 = ADVANCE_MOTION_PATH_MAX_LATERAL_ACC_MM_S2,
    .curvature_preview_mm = ADVANCE_MOTION_PATH_CURVATURE_PREVIEW_MM,
    .curvature_ff_time_s = ADVANCE_MOTION_PATH_CURVATURE_FF_TIME_S,
    .lookahead_min_mm = ADVANCE_MOTION_PATH_LOOKAHEAD_MIN_MM,
    .lookahead_base_mm = ADVANCE_MOTION_PATH_LOOKAHEAD_BASE_MM,
    .lookahead_speed_gain_s = ADVANCE_MOTION_PATH_LOOKAHEAD_SPEED_GAIN_S,
    .lookahead_curve_gain_mm = ADVANCE_MOTION_PATH_LOOKAHEAD_CURVE_GAIN_MM,
    .lookahead_max_mm = ADVANCE_MOTION_PATH_LOOKAHEAD_MAX_MM,
    .lookahead_rate_mm_s = ADVANCE_MOTION_PATH_LOOKAHEAD_RATE_MM_S,
    .initial_lookahead_mm = ADVANCE_MOTION_PATH_INITIAL_LOOKAHEAD_MM,
    .final_capture_distance_mm = ADVANCE_MOTION_PATH_FINAL_CAPTURE_DISTANCE_MM,
    .final_capture_speed_mm_s = ADVANCE_MOTION_PATH_FINAL_CAPTURE_SPEED_MM_S};

/* 返回浮点数的绝对值。 */
static float AdvanceMotion_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void AdvanceMotion_ClearPathContext(void)
{
  g_path = (AdvanceMotion_PathContext_t){0};
}

/* 清除位置 PID 与平面速度估计项。 */
static void AdvanceMotion_ClearLinearControlTerms(void)
{
  g_motion_control.pid_integral_x_mm_s = 0.0f;
  g_motion_control.pid_integral_y_mm_s = 0.0f;
  g_motion_control.pid_last_x_mm = 0.0f;
  g_motion_control.pid_last_y_mm = 0.0f;
  g_motion_control.measured_vx_world_mm_s = 0.0f;
  g_motion_control.measured_vy_world_mm_s = 0.0f;
  g_motion_control.pid_history_valid = 0U;
}

static void AdvanceMotion_ClearYawControlTerms(void)
{
  g_motion_control.last_yaw_updated_tick = 0U;
  g_motion_control.pid_integral_yaw_deg_s = 0.0f;
  g_motion_control.pid_last_yaw_deg = 0.0f;
  g_motion_control.measured_wz_deg_s = 0.0f;
  g_motion_control.pid_history_valid = 0U;
}

static void AdvanceMotion_ClearProgressMonitor(void)
{
  g_motion_control.no_progress_start_tick = 0U;
  g_motion_control.no_progress_reference_error_mm = 0.0f;
}

/* 清除一轮位姿或路径任务的 PID、速度估计与进度校验历史。 */
static void AdvanceMotion_ResetPidAndProgress(void)
{
  g_motion_control.pid_last_tick = 0U;
  g_motion_control.last_pose_updated_tick = 0U;
  AdvanceMotion_ClearLinearControlTerms();
  AdvanceMotion_ClearYawControlTerms();
  AdvanceMotion_ClearProgressMonitor();
  g_motion_control.command_vx_world_mm_s = 0.0f;
  g_motion_control.command_vy_world_mm_s = 0.0f;
  g_motion_control.command_wz_ccw_deg_s = 0.0f;
  g_motion_control.yaw_aligning = 0U;
}

/* Reset the linear loop when crossing between alignment and coupled motion. */
static void AdvanceMotion_ResetLinearPid(void)
{
  g_motion_control.no_progress_start_tick = 0U;
  AdvanceMotion_ClearLinearControlTerms();
}

static float AdvanceMotion_GetLargeYawAlignLinearScale(float yaw_error_deg)
{
  float ratio = AdvanceMotion_AbsFloat(yaw_error_deg) /
                ADVANCE_MOTION_LARGE_YAW_ALIGN_ENTER_DEG;

  ratio = AdvanceWorld_LimitFloat(ratio, 0.0f, 1.0f);
  return 1.0f - ((1.0f - ADVANCE_MOTION_LARGE_YAW_ALIGN_LINEAR_MIN_SCALE) * ratio);
}

/* 读取当前航向源对应的有效标志与更新时间戳，避免跨源配对。 */
static uint8_t AdvanceMotion_GetSelectedYawState(const WorldPose2D_t *pose,
                                                 uint8_t *valid,
                                                 uint32_t *updated_tick)
{
  if ((pose == NULL) || (valid == NULL) || (updated_tick == NULL))
  {
    return 0U;
  }

  if (AdvanceWorld_GetYawSource() == ADVANCE_WORLD_YAW_SOURCE_OPS)
  {
    *valid = pose->ops_yaw_valid;
    *updated_tick = pose->ops_yaw_updated_tick;
  }
  else
  {
    *valid = pose->wit_yaw_valid;
    *updated_tick = pose->wit_yaw_updated_tick;
  }

  return 1U;
}

static uint8_t AdvanceMotion_GetFreshnessFlags(const WorldPose2D_t *pose, uint32_t now_tick)
{
  uint8_t flags = 0U;
  uint8_t yaw_valid;
  uint32_t yaw_updated_tick;

  if (pose == NULL)
  {
    return 0U;
  }

  if ((pose->origin_ready != 0U) &&
      ((now_tick - pose->updated_tick) <= ADVANCE_MOTION_POSE_TIMEOUT_MS))
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH;
  }

  if ((AdvanceMotion_GetSelectedYawState(pose, &yaw_valid, &yaw_updated_tick) != 0U) &&
      (yaw_valid != 0U) &&
      ((now_tick - yaw_updated_tick) <= ADVANCE_MOTION_YAW_TIMEOUT_MS))
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH;
  }

  return flags;
}

static void AdvanceMotion_UpdateDebugSnapshot(uint32_t now_tick, uint8_t flags)
{
  uint8_t freshness_flags;

  if (g_path.active != 0U)
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_PATH_ACTIVE;
  }
  if (AdvanceWorld_GetYawSource() == ADVANCE_WORLD_YAW_SOURCE_OPS)
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_YAW_SOURCE_OPS;
  }
  freshness_flags = AdvanceMotion_GetFreshnessFlags(&g_motion.pose, now_tick);
  flags &= (uint8_t)~(ADVANCE_MOTION_DEBUG_FLAG_VALID |
                      ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH |
                      ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH);
  flags |= freshness_flags;
  if ((g_motion.pose.valid != 0U) &&
      (freshness_flags == (ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH |
                           ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH)))
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_VALID;
  }
  g_motion_debug.tick = now_tick;
  g_motion_debug.pid_revision = g_pid_active_revision;
  g_motion_debug.path_config_revision = g_path_config_active_revision;
  g_motion_debug.state = g_motion_state;
  g_motion_debug.flags = flags;
  g_motion_debug.goal = g_motion.goal;
  g_motion_debug.pose = g_motion.pose;
  g_motion_debug.error_x_mm = g_motion.error_x_mm;
  g_motion_debug.error_y_mm = g_motion.error_y_mm;
  g_motion_debug.error_yaw_deg = g_motion.yaw_error_deg;
  g_motion_debug.command_vx_world_mm_s = g_motion_control.command_vx_world_mm_s;
  g_motion_debug.command_vy_world_mm_s = g_motion_control.command_vy_world_mm_s;
  g_motion_debug.command_wz_ccw_deg_s = g_motion_control.command_wz_ccw_deg_s;
  g_motion_debug.measured_vx_world_mm_s = g_motion_control.measured_vx_world_mm_s;
  g_motion_debug.measured_vy_world_mm_s = g_motion_control.measured_vy_world_mm_s;
  g_motion_debug.measured_wz_deg_s = g_motion_control.measured_wz_deg_s;
  g_motion_debug.integral_x_mm_s = g_motion_control.pid_integral_x_mm_s;
  g_motion_debug.integral_y_mm_s = g_motion_control.pid_integral_y_mm_s;
  g_motion_debug.integral_yaw_deg_s = g_motion_control.pid_integral_yaw_deg_s;
  g_motion_debug.nearest_segment_index = g_path.nearest_index;
  g_motion_debug.target_segment_index = g_path.target_index;
  g_motion_debug.path_progress_mm = g_path.progress_mm;
  g_motion_debug.path_remaining_mm = g_path.remaining_mm;
  g_motion_debug.path_projection_x_mm = g_path.projection_x_mm;
  g_motion_debug.path_projection_y_mm = g_path.projection_y_mm;
  g_motion_debug.path_lookahead_x_mm = g_motion.goal.x_mm;
  g_motion_debug.path_lookahead_y_mm = g_motion.goal.y_mm;
  g_motion_debug.path_signed_curvature_1_mm = g_path.signed_curvature_1_mm;
  g_motion_debug.path_curvature_preview_1_mm = g_path.curvature_preview_1_mm;
  g_motion_debug.path_yaw_gradient_deg_per_mm = g_path.yaw_gradient_deg_per_mm;
  g_motion_debug.path_reference_speed_mm_s = g_path.reference_speed_mm_s;
  g_motion_debug.path_lookahead_mm = g_path.lookahead_mm;
  g_motion_debug.path_feedforward_vx_mm_s = g_path.feedforward_vx_mm_s;
  g_motion_debug.path_feedforward_vy_mm_s = g_path.feedforward_vy_mm_s;
  g_motion_debug.path_feedforward_wz_deg_s = g_path.feedforward_wz_deg_s;
  g_motion_debug.path_cross_track_mm = g_path.cross_track_mm;
  g_motion_debug.path_measured_normal_velocity_mm_s = g_path.measured_normal_velocity_mm_s;
  g_motion_debug.path_normal_velocity_ff_mm_s = g_path.normal_velocity_ff_mm_s;
  g_motion_debug.path_normal_feedback_mm_s = g_path.normal_feedback_mm_s;
  g_motion_debug.path_command_wz_deg_s = g_path.command_wz_deg_s;
  g_motion_debug.path_final_stage = g_path.final_stage;
}

static uint8_t AdvanceMotion_IsPidConfigValid(const AdvanceMotion_PidConfig_t *config)
{
  if ((config == NULL) ||
      (isfinite(config->kp_pos) == 0) ||
      (isfinite(config->ki_pos) == 0) ||
      (isfinite(config->kd_pos) == 0) ||
      (isfinite(config->kp_yaw) == 0) ||
      (isfinite(config->ki_yaw) == 0) ||
      (isfinite(config->kd_yaw) == 0))
  {
    return 0U;
  }

  return ((config->kp_pos >= 0.0f) && (config->kp_pos <= ADVANCE_MOTION_MAX_KP_POS) &&
          (config->ki_pos >= 0.0f) && (config->ki_pos <= ADVANCE_MOTION_MAX_KI_POS) &&
          (config->kd_pos >= 0.0f) && (config->kd_pos <= ADVANCE_MOTION_MAX_KD_POS) &&
          (config->kp_yaw >= 0.0f) && (config->kp_yaw <= ADVANCE_MOTION_MAX_KP_YAW) &&
          (config->ki_yaw >= 0.0f) && (config->ki_yaw <= ADVANCE_MOTION_MAX_KI_YAW) &&
          (config->kd_yaw >= 0.0f) && (config->kd_yaw <= ADVANCE_MOTION_MAX_KD_YAW))
             ? 1U
             : 0U;
}

/* 仅由 20 ms 控制调度调用，保证整组 PID 在周期边界切换。 */
static void AdvanceMotion_ApplyPendingPidConfig(void)
{
  uint32_t primask;
  uint8_t applied = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  if (g_pid_pending_valid != 0U)
  {
    g_pid_active = g_pid_pending;
    g_pid_active_revision = g_pid_pending_revision;
    g_pid_pending_valid = 0U;
    applied = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (applied != 0U)
  {
    AdvanceMotion_ResetPidAndProgress();
  }
}

static uint8_t AdvanceMotion_IsPathControlConfigValid(
    const AdvanceMotion_PathControlConfig_t *config)
{
  if ((config == NULL) ||
      (isfinite(config->kp_cross_track) == 0) ||
      (isfinite(config->kd_cross_track_velocity) == 0) ||
      (isfinite(config->kp_yaw) == 0) ||
      (isfinite(config->kd_yaw_rate) == 0) ||
      (isfinite(config->cruise_speed_mm_s) == 0) ||
      (isfinite(config->max_yaw_rate_deg_s) == 0) ||
      (isfinite(config->accel_mm_s2) == 0) ||
      (isfinite(config->decel_mm_s2) == 0) ||
      (isfinite(config->max_lateral_accel_mm_s2) == 0) ||
      (isfinite(config->curvature_preview_mm) == 0) ||
      (isfinite(config->curvature_ff_time_s) == 0) ||
      (isfinite(config->lookahead_min_mm) == 0) ||
      (isfinite(config->lookahead_base_mm) == 0) ||
      (isfinite(config->lookahead_speed_gain_s) == 0) ||
      (isfinite(config->lookahead_curve_gain_mm) == 0) ||
      (isfinite(config->lookahead_max_mm) == 0) ||
      (isfinite(config->lookahead_rate_mm_s) == 0) ||
      (isfinite(config->initial_lookahead_mm) == 0) ||
      (isfinite(config->final_capture_distance_mm) == 0) ||
      (isfinite(config->final_capture_speed_mm_s) == 0))
  {
    return 0U;
  }

  return ((config->kp_cross_track >= 0.0f) && (config->kp_cross_track <= 20.0f) &&
          (config->kd_cross_track_velocity >= 0.0f) && (config->kd_cross_track_velocity <= 20.0f) &&
          (config->kp_yaw >= 0.0f) && (config->kp_yaw <= 20.0f) &&
          (config->kd_yaw_rate >= 0.0f) && (config->kd_yaw_rate <= 20.0f) &&
          (config->cruise_speed_mm_s > 0.0f) && (config->cruise_speed_mm_s <= ADVANCE_MOTION_MAX_VMAX_MM_S) &&
          (config->max_yaw_rate_deg_s > 0.0f) && (config->max_yaw_rate_deg_s <= ADVANCE_MOTION_MAX_WMAX_DEG_S) &&
          (config->accel_mm_s2 > 0.0f) && (config->accel_mm_s2 <= 5000.0f) &&
          (config->decel_mm_s2 > 0.0f) && (config->decel_mm_s2 <= 5000.0f) &&
          (config->max_lateral_accel_mm_s2 > 0.0f) && (config->max_lateral_accel_mm_s2 <= 5000.0f) &&
          (config->curvature_preview_mm > 0.0f) && (config->curvature_preview_mm <= 2000.0f) &&
          (config->curvature_ff_time_s >= 0.0f) && (config->curvature_ff_time_s <= 2.0f) &&
          (config->lookahead_min_mm > 0.0f) &&
          (config->lookahead_min_mm <= config->lookahead_base_mm) &&
          (config->lookahead_base_mm <= config->lookahead_max_mm) &&
          (config->lookahead_max_mm <= 1000.0f) &&
          (config->lookahead_speed_gain_s >= 0.0f) && (config->lookahead_speed_gain_s <= 2.0f) &&
          (config->lookahead_curve_gain_mm >= 0.0f) && (config->lookahead_curve_gain_mm <= 1000.0f) &&
          (config->lookahead_rate_mm_s > 0.0f) && (config->lookahead_rate_mm_s <= 2000.0f) &&
          (config->initial_lookahead_mm >= config->lookahead_min_mm) &&
          (config->initial_lookahead_mm <= config->lookahead_max_mm) &&
          (config->final_capture_distance_mm >= 0.0f) &&
          (config->final_capture_distance_mm <= 2000.0f) &&
          (config->final_capture_speed_mm_s >= 0.0f) &&
          (config->final_capture_speed_mm_s <= ADVANCE_MOTION_MAX_VMAX_MM_S))
             ? 1U
             : 0U;
}

/* 路径运行中只切换参数并约束当前前视量，不清空路径进度。 */
static void AdvanceMotion_ApplyPendingPathControlConfig(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (g_path_config_pending_valid != 0U)
  {
    g_path_config_active = g_path_config_pending;
    g_path_config_active_revision = g_path_config_pending_revision;
    g_path_config_pending_valid = 0U;
    if (g_path.active != 0U)
    {
      g_path.lookahead_mm = AdvanceWorld_LimitFloat(
          g_path.lookahead_mm,
          g_path_config_active.lookahead_min_mm,
          g_path_config_active.lookahead_max_mm);
      g_motion.goal.vmax_mm_s = g_path_config_active.cruise_speed_mm_s;
      g_motion.goal.wmax_deg_s = g_path_config_active.max_yaw_rate_deg_s;
    }
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* 按传感器时间戳更新实测速度，并记录控制周期时间。 */
static void AdvanceMotion_SavePidPose(const WorldPose2D_t *pose, uint32_t now_tick)
{
  if (g_motion_control.pid_history_valid == 0U)
  {
    g_motion_control.pid_last_x_mm = pose->x_mm;
    g_motion_control.pid_last_y_mm = pose->y_mm;
    g_motion_control.pid_last_yaw_deg = pose->yaw_deg;
    g_motion_control.last_pose_updated_tick = pose->updated_tick;
    g_motion_control.last_yaw_updated_tick = pose->yaw_updated_tick;
    g_motion_control.measured_vx_world_mm_s = 0.0f;
    g_motion_control.measured_vy_world_mm_s = 0.0f;
    g_motion_control.measured_wz_deg_s = 0.0f;
  }
  else
  {
    if (pose->updated_tick != g_motion_control.last_pose_updated_tick)
    {
      uint32_t pose_dt_ms = pose->updated_tick - g_motion_control.last_pose_updated_tick;

      if (pose_dt_ms > 0U)
      {
        float pose_dt_s = (float)pose_dt_ms / 1000.0f;

        g_motion_control.measured_vx_world_mm_s =
            (pose->x_mm - g_motion_control.pid_last_x_mm) / pose_dt_s;
        g_motion_control.measured_vy_world_mm_s =
            (pose->y_mm - g_motion_control.pid_last_y_mm) / pose_dt_s;
      }
      g_motion_control.pid_last_x_mm = pose->x_mm;
      g_motion_control.pid_last_y_mm = pose->y_mm;
      g_motion_control.last_pose_updated_tick = pose->updated_tick;
    }

    if (pose->yaw_updated_tick != g_motion_control.last_yaw_updated_tick)
    {
      uint32_t yaw_dt_ms = pose->yaw_updated_tick - g_motion_control.last_yaw_updated_tick;

      if (yaw_dt_ms > 0U)
      {
        float yaw_dt_s = (float)yaw_dt_ms / 1000.0f;
        float yaw_delta = AdvanceWorld_WrapAngleDeg(
            pose->yaw_deg - g_motion_control.pid_last_yaw_deg);

        g_motion_control.measured_wz_deg_s = yaw_delta / yaw_dt_s;
      }
      g_motion_control.pid_last_yaw_deg = pose->yaw_deg;
      g_motion_control.last_yaw_updated_tick = pose->yaw_updated_tick;
    }
  }

  g_motion_control.pid_last_tick = now_tick;
  g_motion_control.pid_history_valid = 1U;
}

/* 在速度未朝同方向饱和时累积积分，避免目标较远时积分继续堆积。 */
static void AdvanceMotion_UpdatePidIntegral(float vx_world_mm_s, float vy_world_mm_s,
                                             float wz_ccw_deg_s, float dt_s,
                                             uint8_t linear_saturated, uint8_t yaw_saturated,
                                             uint8_t position_required, uint8_t yaw_required)
{
  if ((position_required != 0U) &&
      ((linear_saturated == 0U) ||
       (((g_motion.error_x_mm * vx_world_mm_s) +
         (g_motion.error_y_mm * vy_world_mm_s)) <= 0.0f)))
  {
    g_motion_control.pid_integral_x_mm_s = AdvanceWorld_LimitFloat(
        g_motion_control.pid_integral_x_mm_s + (g_motion.error_x_mm * dt_s),
        -ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S,
        ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S);
    g_motion_control.pid_integral_y_mm_s = AdvanceWorld_LimitFloat(
        g_motion_control.pid_integral_y_mm_s + (g_motion.error_y_mm * dt_s),
        -ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S,
        ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S);
  }

  if ((yaw_required != 0U) &&
      ((yaw_saturated == 0U) || ((g_motion.yaw_error_deg * wz_ccw_deg_s) <= 0.0f)))
  {
    g_motion_control.pid_integral_yaw_deg_s = AdvanceWorld_LimitFloat(
        g_motion_control.pid_integral_yaw_deg_s + (g_motion.yaw_error_deg * dt_s),
        -ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S,
        ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S);
  }
}

/* 路径中段按累计弧长、Goto 阶段按位置误差校验运动进展。 */
static uint8_t AdvanceMotion_HasNoProgress(uint32_t now_tick, float command_magnitude)
{
  if ((g_path.active != 0U) && (g_path.final_stage == 0U))
  {
    if (command_magnitude < ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S)
    {
      return 0U;
    }
    if ((g_path.progress_mm - g_path.progress_reference_mm) >=
        ADVANCE_MOTION_NO_PROGRESS_MIN_REDUCTION_MM)
    {
      g_path.progress_reference_mm = g_path.progress_mm;
      g_path.progress_tick = now_tick;
      return 0U;
    }
    return ((now_tick - g_path.progress_tick) >= ADVANCE_MOTION_NO_PROGRESS_WINDOW_MS) ? 1U : 0U;
  }

  if ((g_motion.position_error_mm <= ADVANCE_MOTION_POS_TOLERANCE_MM) ||
      (command_magnitude < ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S))
  {
    g_motion_control.no_progress_start_tick = 0U;
    return 0U;
  }

  if (g_motion_control.no_progress_start_tick == 0U)
  {
    g_motion_control.no_progress_start_tick = now_tick;
    g_motion_control.no_progress_reference_error_mm = g_motion.position_error_mm;
    return 0U;
  }

  if ((g_motion_control.no_progress_reference_error_mm - g_motion.position_error_mm) >=
      ADVANCE_MOTION_NO_PROGRESS_MIN_REDUCTION_MM)
  {
    g_motion_control.no_progress_start_tick = now_tick;
    g_motion_control.no_progress_reference_error_mm = g_motion.position_error_mm;
    return 0U;
  }

  return ((now_tick - g_motion_control.no_progress_start_tick) >=
          ADVANCE_MOTION_NO_PROGRESS_WINDOW_MS) ? 1U : 0U;
}

/* 按最大模长限制二维速度向量，同时保持其方向不变。 */
static float AdvanceMotion_LimitVector(float *vx, float *vy, float max_value)
{
  float magnitude;
  float scale;

  if ((vx == 0) || (vy == 0))
  {
    return 0.0f;
  }

  magnitude = sqrtf((*vx * *vx) + (*vy * *vy));
  if ((max_value > 0.0f) && (magnitude > max_value))
  {
    scale = max_value / magnitude;
    *vx *= scale;
    *vy *= scale;
    magnitude = max_value;
  }

  return magnitude;
}

/* 获取目标点的线速度上限，未设置时使用默认值。 */
static float AdvanceMotion_GetGoalVmax(const WorldGoalPose2D_t *goal)
{
  return (goal->vmax_mm_s > 0.0f) ? goal->vmax_mm_s : ADVANCE_MOTION_DEFAULT_VMAX_MM_S;
}

/* 获取目标点的角速度上限，未设置时使用默认值。 */
static float AdvanceMotion_GetGoalWmax(const WorldGoalPose2D_t *goal)
{
  return (goal->wmax_deg_s > 0.0f) ? goal->wmax_deg_s : ADVANCE_MOTION_DEFAULT_WMAX_DEG_S;
}

/* 校验目标位姿、速度上限、超时时间和标志位。 */
static uint8_t AdvanceMotion_IsGoalValid(const WorldGoalPose2D_t *goal)
{
  if ((goal == 0) ||
      (isfinite(goal->x_mm) == 0) ||
      (isfinite(goal->y_mm) == 0) ||
      (isfinite(goal->yaw_deg) == 0) ||
      (isfinite(goal->vmax_mm_s) == 0) ||
      (isfinite(goal->wmax_deg_s) == 0))
  {
    return 0U;
  }

  if ((((goal->goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) != 0U) &&
       ((goal->x_mm < ADVANCE_MOTION_WORLD_X_MIN_MM) ||
        (goal->x_mm > ADVANCE_MOTION_WORLD_X_MAX_MM) ||
        (goal->y_mm < ADVANCE_MOTION_WORLD_Y_MIN_MM) ||
        (goal->y_mm > ADVANCE_MOTION_WORLD_Y_MAX_MM) ||
        (goal->vmax_mm_s <= 0.0f) ||
        (goal->vmax_mm_s > ADVANCE_MOTION_MAX_VMAX_MM_S))) ||
      (((goal->goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U) &&
       ((goal->wmax_deg_s <= 0.0f) ||
        (goal->wmax_deg_s > ADVANCE_MOTION_MAX_WMAX_DEG_S))) ||
      (goal->timeout_ms > ADVANCE_MOTION_MAX_TIMEOUT_MS) ||
      ((goal->goal_flags & (uint8_t)(~(ADVANCE_MOTION_GOAL_USE_YAW | ADVANCE_MOTION_GOAL_USE_POSITION))) != 0U) ||
      ((goal->goal_flags & (ADVANCE_MOTION_GOAL_USE_YAW | ADVANCE_MOTION_GOAL_USE_POSITION)) == 0U))
  {
    return 0U;
  }

  return 1U;
}

static float AdvanceMotion_GetPathSegmentLength(uint16_t index)
{
  float dx = g_path.points[index + 1U].x_mm - g_path.points[index].x_mm;
  float dy = g_path.points[index + 1U].y_mm - g_path.points[index].y_mm;

  return sqrtf((dx * dx) + (dy * dy));
}

/* 一次评估路径顶点的带符号和绝对曲率，单位均为 1/mm。 */
static AdvanceMotion_PathVertexCurvature_t AdvanceMotion_EvaluatePathVertexCurvature(uint16_t index)
{
  AdvanceMotion_PathVertexCurvature_t curvature = {0.0f, 0.0f};
  const AdvanceMotion_PathPoint_t *a;
  const AdvanceMotion_PathPoint_t *b;
  const AdvanceMotion_PathPoint_t *c;
  float abx;
  float aby;
  float bcx;
  float bcy;
  float acx;
  float acy;
  float ab;
  float bc;
  float ac;
  float cross;

  if ((g_path.points == NULL) || (g_path.point_count < 3U) ||
      (index == 0U) || (index >= (uint16_t)(g_path.point_count - 1U)))
  {
    return curvature;
  }

  a = &g_path.points[index - 1U];
  b = &g_path.points[index];
  c = &g_path.points[index + 1U];
  abx = b->x_mm - a->x_mm;
  aby = b->y_mm - a->y_mm;
  bcx = c->x_mm - b->x_mm;
  bcy = c->y_mm - b->y_mm;
  acx = c->x_mm - a->x_mm;
  acy = c->y_mm - a->y_mm;
  ab = sqrtf((abx * abx) + (aby * aby));
  bc = sqrtf((bcx * bcx) + (bcy * bcy));
  ac = sqrtf((acx * acx) + (acy * acy));
  cross = (abx * bcy) - (aby * bcx);

  if ((ab < ADVANCE_MOTION_PATH_MIN_SEGMENT_MM) ||
      (bc < ADVANCE_MOTION_PATH_MIN_SEGMENT_MM))
  {
    return curvature;
  }

  if (ac < ADVANCE_MOTION_PATH_MIN_SEGMENT_MM)
  {
    curvature.absolute_curvature_1_mm = 1.0f / ADVANCE_MOTION_PATH_MIN_SEGMENT_MM;
    return curvature;
  }
  curvature.signed_curvature_1_mm = (2.0f * cross) / (ab * bc * ac);
  curvature.absolute_curvature_1_mm = AdvanceMotion_AbsFloat(curvature.signed_curvature_1_mm);
  return curvature;
}

static void AdvanceMotion_UpdatePathPreview(void)
{
  uint16_t segment_index;
  uint16_t vertex_index;
  float distance_to_segment_start_mm = 0.0f;
  float distance_to_vertex_mm;
  float current_length = AdvanceMotion_GetPathSegmentLength(g_path.nearest_index);

  g_path.curvature_preview_1_mm = 0.0f;
  g_path.yaw_gradient_preview_deg_per_mm = 0.0f;
  for (segment_index = g_path.nearest_index;
       segment_index < (uint16_t)(g_path.point_count - 1U);
       ++segment_index)
  {
    float length = AdvanceMotion_GetPathSegmentLength(segment_index);
    float gradient = AdvanceMotion_AbsFloat(AdvanceWorld_WrapAngleDeg(
                         g_path.points[segment_index + 1U].yaw_deg -
                         g_path.points[segment_index].yaw_deg)) /
                     length;

    if (distance_to_segment_start_mm > g_path_config_active.curvature_preview_mm)
    {
      break;
    }
    g_path.yaw_gradient_preview_deg_per_mm =
        fmaxf(g_path.yaw_gradient_preview_deg_per_mm, gradient);
    distance_to_segment_start_mm += (segment_index == g_path.nearest_index)
                                        ? ((1.0f - g_path.progress_on_segment) * current_length)
                                        : length;
  }

  distance_to_vertex_mm = (1.0f - g_path.progress_on_segment) * current_length;
  for (vertex_index = (uint16_t)(g_path.nearest_index + 1U);
       vertex_index < (uint16_t)(g_path.point_count - 1U);
       ++vertex_index)
  {
    if (distance_to_vertex_mm > g_path_config_active.curvature_preview_mm)
    {
      break;
    }
    g_path.curvature_preview_1_mm = fmaxf(
        g_path.curvature_preview_1_mm,
        AdvanceMotion_EvaluatePathVertexCurvature(vertex_index).absolute_curvature_1_mm);
    distance_to_vertex_mm += AdvanceMotion_GetPathSegmentLength(vertex_index);
  }
}

static float AdvanceMotion_GetPathSpeedLimit(void)
{
  float curvature_speed = sqrtf(
      g_path_config_active.max_lateral_accel_mm_s2 /
      fmaxf(g_path.curvature_preview_1_mm,
            ADVANCE_MOTION_PATH_CURVATURE_EPSILON_1_MM));
  float yaw_speed = g_path_config_active.max_yaw_rate_deg_s /
                    fmaxf(g_path.yaw_gradient_preview_deg_per_mm,
                          ADVANCE_MOTION_PATH_YAW_GRADIENT_EPSILON_DEG_PER_MM);
  float braking_distance = fmaxf(
      g_path.remaining_mm - g_path_config_active.final_capture_distance_mm, 0.0f);
  float final_speed = sqrtf(
      (g_path_config_active.final_capture_speed_mm_s *
       g_path_config_active.final_capture_speed_mm_s) +
      (2.0f * g_path_config_active.decel_mm_s2 * braking_distance));

  return fminf(g_path_config_active.cruise_speed_mm_s,
               fminf(curvature_speed, fminf(yaw_speed, final_speed)));
}

static void AdvanceMotion_UpdatePathLookahead(float dt_s)
{
  float curvature_ratio = AdvanceWorld_LimitFloat(
      g_path.curvature_preview_1_mm * g_path_config_active.curvature_preview_mm,
      0.0f, 1.0f);
  float target = g_path_config_active.lookahead_base_mm +
                 (g_path_config_active.lookahead_speed_gain_s *
                  g_path.reference_speed_mm_s) -
                 (g_path_config_active.lookahead_curve_gain_mm * curvature_ratio);
  float max_delta = g_path_config_active.lookahead_rate_mm_s * dt_s;

  target = AdvanceWorld_LimitFloat(target,
                                   g_path_config_active.lookahead_min_mm,
                                   g_path_config_active.lookahead_max_mm);
  g_path.lookahead_mm = AdvanceWorld_LimitFloat(
      target, g_path.lookahead_mm - max_delta, g_path.lookahead_mm + max_delta);
}

static void AdvanceMotion_SetPathLookaheadGoal(void)
{
  uint16_t index = g_path.nearest_index;
  float segment_length = AdvanceMotion_GetPathSegmentLength(index);
  float distance_mm = g_path.lookahead_mm;
  float available_mm = (1.0f - g_path.progress_on_segment) * segment_length;
  float t;

  if (distance_mm <= available_mm)
  {
    t = g_path.progress_on_segment + (distance_mm / segment_length);
  }
  else
  {
    distance_mm -= available_mm;
    ++index;
    while (index < (uint16_t)(g_path.point_count - 1U))
    {
      segment_length = AdvanceMotion_GetPathSegmentLength(index);
      if (distance_mm <= segment_length)
      {
        break;
      }
      distance_mm -= segment_length;
      ++index;
    }
    if (index >= (uint16_t)(g_path.point_count - 1U))
    {
      index = (uint16_t)(g_path.point_count - 2U);
      segment_length = AdvanceMotion_GetPathSegmentLength(index);
      t = 1.0f;
    }
    else
    {
      t = distance_mm / segment_length;
    }
  }

  g_path.target_index = index;
  g_motion.goal.x_mm = g_path.points[index].x_mm +
                       (t * (g_path.points[index + 1U].x_mm - g_path.points[index].x_mm));
  g_motion.goal.y_mm = g_path.points[index].y_mm +
                       (t * (g_path.points[index + 1U].y_mm - g_path.points[index].y_mm));
  g_path.feedforward_vx_mm_s =
      ((g_path.points[index + 1U].x_mm - g_path.points[index].x_mm) / segment_length) *
      g_path.reference_speed_mm_s;
  g_path.feedforward_vy_mm_s =
      ((g_path.points[index + 1U].y_mm - g_path.points[index].y_mm) / segment_length) *
      g_path.reference_speed_mm_s;
}

static void AdvanceMotion_UpdatePathReference(uint32_t now_tick)
{
  uint16_t search_start_index = g_path.nearest_index;
  uint16_t end_index;
  uint16_t index;
  uint16_t best_index = search_start_index;
  float best_t = g_path.progress_on_segment;
  float best_distance_squared = 0.0f;
  float reference_dt_s = (float)(now_tick - g_path.reference_tick) / 1000.0f;
  float target_speed;
  float speed_delta;
  float measured_speed;

  end_index = search_start_index + ADVANCE_MOTION_PATH_SEARCH_SEGMENTS;
  if ((end_index < search_start_index) ||
      (end_index >= (uint16_t)(g_path.point_count - 1U)))
  {
    end_index = (uint16_t)(g_path.point_count - 2U);
  }
  for (index = search_start_index; index <= end_index; ++index)
  {
    const AdvanceMotion_PathPoint_t *a = &g_path.points[index];
    const AdvanceMotion_PathPoint_t *b = &g_path.points[index + 1U];
    float dx = b->x_mm - a->x_mm;
    float dy = b->y_mm - a->y_mm;
    float len2 = (dx * dx) + (dy * dy);
    float t = ((g_motion.pose.x_mm - a->x_mm) * dx +
               (g_motion.pose.y_mm - a->y_mm) * dy) /
              len2;
    float ex;
    float ey;
    float d2;

    t = AdvanceWorld_LimitFloat(t,
                                (index == search_start_index) ? g_path.progress_on_segment : 0.0f,
                                1.0f);
    ex = g_motion.pose.x_mm - (a->x_mm + (t * dx));
    ey = g_motion.pose.y_mm - (a->y_mm + (t * dy));
    d2 = (ex * ex) + (ey * ey);
    if ((index == search_start_index) || (d2 < best_distance_squared))
    {
      best_index = index;
      best_t = t;
      best_distance_squared = d2;
    }
  }

  for (index = g_path.nearest_index; index < best_index; ++index)
  {
    g_path.completed_length_mm += AdvanceMotion_GetPathSegmentLength(index);
  }
  g_path.nearest_index = best_index;
  g_path.progress_on_segment = best_t;
  {
    const AdvanceMotion_PathPoint_t *a = &g_path.points[best_index];
    const AdvanceMotion_PathPoint_t *b = &g_path.points[best_index + 1U];
    float dx = b->x_mm - a->x_mm;
    float dy = b->y_mm - a->y_mm;
    float length = sqrtf((dx * dx) + (dy * dy));
    float ex;
    float ey;

    g_path.projection_x_mm = a->x_mm + (best_t * dx);
    g_path.projection_y_mm = a->y_mm + (best_t * dy);
    ex = g_motion.pose.x_mm - g_path.projection_x_mm;
    ey = g_motion.pose.y_mm - g_path.projection_y_mm;
    g_path.cross_track_mm = ((dx * ey) - (dy * ex)) / length;
    g_path.progress_mm = g_path.completed_length_mm + (best_t * length);
    g_path.remaining_mm = fmaxf(g_path.total_length_mm - g_path.progress_mm, 0.0f);
    g_path.yaw_gradient_deg_per_mm =
        AdvanceWorld_WrapAngleDeg(b->yaw_deg - a->yaw_deg) / length;
    g_motion.goal.yaw_deg = AdvanceWorld_WrapAngleDeg(
        a->yaw_deg + (best_t * AdvanceWorld_WrapAngleDeg(b->yaw_deg - a->yaw_deg)));
  }

  g_path.signed_curvature_1_mm = 0.0f;
  if (g_path.point_count >= 3U)
  {
    uint16_t vertex_index = (uint16_t)(g_path.nearest_index + 1U);

    if (vertex_index >= (uint16_t)(g_path.point_count - 1U))
    {
      vertex_index = (uint16_t)(g_path.point_count - 2U);
    }
    g_path.signed_curvature_1_mm =
        AdvanceMotion_EvaluatePathVertexCurvature(vertex_index).signed_curvature_1_mm;
  }

  AdvanceMotion_UpdatePathPreview();
  target_speed = AdvanceMotion_GetPathSpeedLimit();
  speed_delta = ((target_speed >= g_path.reference_speed_mm_s)
                     ? g_path_config_active.accel_mm_s2
                     : g_path_config_active.decel_mm_s2) *
                reference_dt_s;
  g_path.reference_speed_mm_s = AdvanceWorld_LimitFloat(
      target_speed,
      fmaxf(g_path.reference_speed_mm_s - speed_delta, 0.0f),
      g_path.reference_speed_mm_s + speed_delta);
  AdvanceMotion_UpdatePathLookahead(reference_dt_s);
  AdvanceMotion_SetPathLookaheadGoal();
  g_path.feedforward_wz_deg_s =
      g_path.yaw_gradient_deg_per_mm * g_path.reference_speed_mm_s;
  g_path.reference_tick = now_tick;

  measured_speed = sqrtf(
      (g_motion_control.measured_vx_world_mm_s * g_motion_control.measured_vx_world_mm_s) +
      (g_motion_control.measured_vy_world_mm_s * g_motion_control.measured_vy_world_mm_s));
  if ((g_path.final_stage == 0U) &&
      (g_path.remaining_mm <= g_path_config_active.final_capture_distance_mm) &&
      (g_path.reference_speed_mm_s <= g_path_config_active.final_capture_speed_mm_s) &&
      (measured_speed <= g_path_config_active.final_capture_speed_mm_s))
  {
    g_path.final_stage = 1U;
    g_motion_control.pid_integral_x_mm_s = 0.0f;
    g_motion_control.pid_integral_y_mm_s = 0.0f;
    g_motion_control.pid_integral_yaw_deg_s = 0.0f;
    g_motion_control.no_progress_start_tick = 0U;
  }
  if (g_path.final_stage != 0U)
  {
    const AdvanceMotion_PathPoint_t *final_point = &g_path.points[g_path.point_count - 1U];

    g_motion.goal.x_mm = final_point->x_mm;
    g_motion.goal.y_mm = final_point->y_mm;
    g_motion.goal.yaw_deg = final_point->yaw_deg;
    g_path.feedforward_vx_mm_s = 0.0f;
    g_path.feedforward_vy_mm_s = 0.0f;
    g_path.feedforward_wz_deg_s = 0.0f;
  }
}

static AdvanceMotion_Status_t AdvanceMotion_GetFreshYaw(WorldPose2D_t *pose)
{
  float yaw_deg;
  uint32_t updated_tick;

  if (pose == NULL)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  (void)AdvanceWorld_GetPoseCopy(pose);
  if (AdvanceWorld_GetYawCopy(&yaw_deg, &updated_tick) != ADVANCE_WORLD_STATUS_OK)
  {
    return ADVANCE_MOTION_STATUS_NO_POSE;
  }
  if ((HAL_GetTick() - updated_tick) > ADVANCE_MOTION_YAW_TIMEOUT_MS)
  {
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }
  pose->yaw_deg = yaw_deg;
  pose->yaw_updated_tick = updated_tick;
  return ADVANCE_MOTION_STATUS_OK;
}

/* 获取未超时的有效世界坐标，并转换为运动控制状态码。 */
static AdvanceMotion_Status_t AdvanceMotion_GetFreshPose(WorldPose2D_t *pose, uint32_t now_tick)
{
  AdvanceWorld_Status_t world_status;
  uint8_t yaw_valid;
  uint32_t yaw_updated_tick;

  if (pose == 0)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  world_status = AdvanceWorld_GetPoseCopy(pose);
  if (world_status == ADVANCE_WORLD_STATUS_NO_ORIGIN)
  {
    return ADVANCE_MOTION_STATUS_NO_ORIGIN;
  }

  if ((world_status != ADVANCE_WORLD_STATUS_OK) || (pose->valid == 0U))
  {
    return ADVANCE_MOTION_STATUS_NO_POSE;
  }

  if ((now_tick - pose->updated_tick) > ADVANCE_MOTION_POSE_TIMEOUT_MS)
  {
    /* OPS 位置时间戳超时，世界线速度无法安全转换到车体坐标。 */
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }

  if ((AdvanceMotion_GetSelectedYawState(pose, &yaw_valid, &yaw_updated_tick) == 0U) ||
      (yaw_valid == 0U))
  {
    /* 当前航向源无有效样本，即使 OPS 位置仍新鲜也不能继续世界坐标控制。 */
    return ADVANCE_MOTION_STATUS_NO_POSE;
  }
  if ((now_tick - yaw_updated_tick) > ADVANCE_MOTION_YAW_TIMEOUT_MS)
  {
    /* 当前航向时间戳超时，禁止继续世界速度到车体速度的转换。 */
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }

  return ADVANCE_MOTION_STATUS_OK;
}

static void AdvanceMotion_UpdateInactiveDebugSnapshot(uint32_t now_tick)
{
  uint8_t flags = 0U;

  if (AdvanceMotion_GetFreshPose(&g_motion.pose, now_tick) == ADVANCE_MOTION_STATUS_OK)
  {
    g_motion.error_x_mm = g_motion.goal.x_mm - g_motion.pose.x_mm;
    g_motion.error_y_mm = g_motion.goal.y_mm - g_motion.pose.y_mm;
    g_motion.position_error_mm = sqrtf((g_motion.error_x_mm * g_motion.error_x_mm) +
                                       (g_motion.error_y_mm * g_motion.error_y_mm));
    g_motion.yaw_error_deg = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U)
                                 ? AdvanceWorld_WrapAngleDeg(g_motion.goal.yaw_deg - g_motion.pose.yaw_deg)
                                 : 0.0f;
  }
  if ((g_motion_control.terminal_stop_pending != 0U) &&
      (Chassis_SmoothStop(g_motion_control.acc) != 0U))
  {
    g_motion_control.terminal_stop_pending = 0U;
  }
  AdvanceMotion_UpdateDebugSnapshot(now_tick, flags);
}

static void AdvanceMotion_SetTerminalState(AdvanceMotion_RunState_t state)
{
  if (g_motion_control.arrival_stop_sent == 0U)
  {
    g_motion_control.terminal_stop_pending =
        (Chassis_SmoothStop(g_motion_control.acc) != 0U) ? 0U : 1U;
  }
  g_motion.updated_tick = HAL_GetTick();
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  g_motion_control.arrival_hard_stop_sent = 0U;
  AdvanceMotion_ResetPidAndProgress();
  AdvanceMotion_ClearPathContext();
  (void)AdvanceControl_ReleaseMode();
  g_motion_state = state;
}

static AdvanceMotion_Status_t AdvanceMotion_ApplyWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc, const WorldPose2D_t *pose)
{
  WorldPose2D_t current_pose;
  float vx_body_mm_s;
  float vy_body_mm_s;
  AdvanceMotion_Status_t pose_status;

  if (pose == NULL)
  {
    pose_status = AdvanceMotion_GetFreshPose(&current_pose, HAL_GetTick());
    if (pose_status == ADVANCE_MOTION_STATUS_NO_ORIGIN)
    {
      Chassis_SmoothStop(acc);
      return ADVANCE_MOTION_STATUS_NO_ORIGIN;
    }

    if (pose_status == ADVANCE_MOTION_STATUS_NO_POSE)
    {
      Chassis_SmoothStop(acc);
      return ADVANCE_MOTION_STATUS_NO_POSE;
    }

    if (pose_status == ADVANCE_MOTION_STATUS_POSE_TIMEOUT)
    {
      Chassis_SmoothStop(acc);
      return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
    }

    if (pose_status != ADVANCE_MOTION_STATUS_OK)
    {
      Chassis_SmoothStop(acc);
      return pose_status;
    }

    pose = &current_pose;
  }

  AdvanceWorld_WorldToBodyVelocity(vx_world_mm_s, vy_world_mm_s, pose->yaw_deg, &vx_body_mm_s, &vy_body_mm_s);
  return (Chassis_SetBodyVelocityEx(vx_body_mm_s, vy_body_mm_s, wz_ccw_deg_s, acc) != 0U)
             ? ADVANCE_MOTION_STATUS_OK
             : ADVANCE_MOTION_STATUS_BUSY;
}

/* 初始化世界坐标运动控制器。 */
void AdvanceMotion_Init(void)
{
  g_motion = (AdvanceMotion_RuntimeData_t){0};
  g_motion_control = (AdvanceMotion_Control_t){0};
  AdvanceMotion_ClearPathContext();
  g_motion_state = ADVANCE_MOTION_STATE_IDLE;
  g_pid_active = g_pid_default;
  g_pid_pending = g_pid_default;
  g_pid_pending_valid = 0U;
  g_pid_active_revision = 0U;
  g_pid_pending_revision = 0U;
  g_pid_next_revision = 0U;
  g_path_config_active = g_path_config_default;
  g_path_config_pending = g_path_config_default;
  g_path_config_pending_valid = 0U;
  g_path_config_active_revision = 0U;
  g_path_config_pending_revision = 0U;
  g_path_config_next_revision = 0U;
  g_large_yaw_align_enabled = ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE;
  AdvanceMotion_UpdateDebugSnapshot(HAL_GetTick(), 0U);
}

/* 设置世界坐标系速度，并取消正在执行的位姿或路径任务。 */
AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  if (AdvanceControl_GetMode() == ADVANCE_CONTROL_VISUAL)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    g_motion_state = ADVANCE_MOTION_STATE_CANCELED;
    g_motion.updated_tick = HAL_GetTick();
    g_motion_control.arrive_hold_start_tick = 0U;
    g_motion_control.arrival_stop_sent = 0U;
    g_motion_control.arrival_hard_stop_sent = 0U;
    AdvanceMotion_ResetPidAndProgress();
    AdvanceMotion_ClearPathContext();
  }

  return AdvanceMotion_ApplyWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc, NULL);
}

/* 设置目标位姿并启动闭环到点运动任务。 */
AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc)
{
  WorldPose2D_t pose;
  AdvanceMotion_Status_t pose_status;

  if (AdvanceMotion_IsGoalValid(goal) == 0U)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }

  pose_status = ((goal->goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) != 0U)
                    ? AdvanceMotion_GetFreshPose(&pose, HAL_GetTick())
                    : AdvanceMotion_GetFreshYaw(&pose);
  if (pose_status != ADVANCE_MOTION_STATUS_OK)
  {
    return pose_status;
  }
  if (AdvanceControl_SetMode(ADVANCE_CONTROL_WORLD) == 0U)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }

  g_motion.goal = *goal;
  g_motion.pose = pose;
  g_motion.started_tick = HAL_GetTick();
  g_motion.updated_tick = g_motion.started_tick;
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  g_motion_control.arrival_hard_stop_sent = 0U;
  AdvanceMotion_ResetPidAndProgress();
  g_motion.error_x_mm = 0.0f;
  g_motion.error_y_mm = 0.0f;
  g_motion.position_error_mm = 0.0f;
  g_motion.yaw_error_deg = 0.0f;
  g_motion_control.acc = acc;
  g_motion_control.large_yaw_align_enabled = g_large_yaw_align_enabled;
  g_motion_control.yaw_aligning =
      ((g_motion_control.large_yaw_align_enabled != 0U) &&
       ((goal->goal_flags & (ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW)) ==
        (ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW)) &&
       (AdvanceMotion_AbsFloat(AdvanceWorld_WrapAngleDeg(goal->yaw_deg - pose.yaw_deg)) >=
        ADVANCE_MOTION_LARGE_YAW_ALIGN_ENTER_DEG)) ? 1U : 0U;
  AdvanceMotion_SavePidPose(&g_motion.pose, g_motion.started_tick);
  g_motion_state = ADVANCE_MOTION_STATE_RUNNING;
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_FollowPathEx(const AdvanceMotion_PathPoint_t *points,
                                                   uint16_t point_count)
{
  WorldPose2D_t pose;
  AdvanceMotion_Status_t pose_status;
  uint16_t index;
  float total_length_mm = 0.0f;
  float minimum_segment_squared = ADVANCE_MOTION_PATH_MIN_SEGMENT_MM *
                                  ADVANCE_MOTION_PATH_MIN_SEGMENT_MM;

  if ((points == NULL) || (point_count < 2U))
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  for (index = 0U; index < point_count; ++index)
  {
    if ((isfinite(points[index].x_mm) == 0) || (isfinite(points[index].y_mm) == 0) ||
        (isfinite(points[index].yaw_deg) == 0) ||
        (points[index].x_mm < ADVANCE_MOTION_WORLD_X_MIN_MM) ||
        (points[index].x_mm > ADVANCE_MOTION_WORLD_X_MAX_MM) ||
        (points[index].y_mm < ADVANCE_MOTION_WORLD_Y_MIN_MM) ||
        (points[index].y_mm > ADVANCE_MOTION_WORLD_Y_MAX_MM) ||
        ((index > 0U) &&
         (((points[index].x_mm - points[index - 1U].x_mm) * (points[index].x_mm - points[index - 1U].x_mm)) +
          ((points[index].y_mm - points[index - 1U].y_mm) * (points[index].y_mm - points[index - 1U].y_mm)) <
          minimum_segment_squared)))
    {
      return ADVANCE_MOTION_STATUS_INVALID_PARAM;
    }
    if (index > 0U)
    {
      float dx = points[index].x_mm - points[index - 1U].x_mm;
      float dy = points[index].y_mm - points[index - 1U].y_mm;

      total_length_mm += sqrtf((dx * dx) + (dy * dy));
    }
  }
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }

  pose_status = AdvanceMotion_GetFreshPose(&pose, HAL_GetTick());
  if (pose_status != ADVANCE_MOTION_STATUS_OK)
  {
    return pose_status;
  }
  if (AdvanceControl_SetMode(ADVANCE_CONTROL_WORLD) == 0U)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }

  AdvanceMotion_ClearPathContext();
  g_path.points = points;
  g_path.point_count = point_count;
  g_path.progress_tick = HAL_GetTick();
  g_path.reference_tick = g_path.progress_tick;
  g_path.total_length_mm = total_length_mm;
  g_path.remaining_mm = total_length_mm;
  g_path.progress_reference_mm = 0.0f;
  g_path.lookahead_mm = AdvanceWorld_LimitFloat(
      g_path_config_active.initial_lookahead_mm,
      g_path_config_active.lookahead_min_mm,
      g_path_config_active.lookahead_max_mm);
  {
    float timeout_ms = (float)ADVANCE_MOTION_PATH_TIMEOUT_BASE_MS +
                       ((total_length_mm /
                         ADVANCE_MOTION_PATH_TIMEOUT_EXPECTED_MIN_SPEED_MM_S) *
                        1000.0f * ADVANCE_MOTION_PATH_TIMEOUT_SCALE);

    g_path.timeout_ms = (uint32_t)fminf(timeout_ms,
                                        (float)ADVANCE_MOTION_PATH_TIMEOUT_MAX_MS);
  }
  g_path.active = 1U;
  g_motion.goal = (WorldGoalPose2D_t){0};
  g_motion.goal.vmax_mm_s = g_path_config_active.cruise_speed_mm_s;
  g_motion.goal.wmax_deg_s = g_path_config_active.max_yaw_rate_deg_s;
  g_motion.goal.timeout_ms = g_path.timeout_ms;
  g_motion.goal.goal_flags = ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW;
  g_motion.pose = pose;
  g_motion.started_tick = g_path.progress_tick;
  g_motion.updated_tick = g_motion.started_tick;
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  g_motion_control.arrival_hard_stop_sent = 0U;
  AdvanceMotion_ResetPidAndProgress();
  AdvanceMotion_UpdatePathReference(g_path.progress_tick);
  g_motion.error_x_mm = 0.0f;
  g_motion.error_y_mm = 0.0f;
  g_motion.position_error_mm = 0.0f;
  g_motion.yaw_error_deg = 0.0f;
  g_motion_control.acc = ADVANCE_MOTION_PATH_DRIVER_ACC;
  /* 路径任务必须持续平移与旋转并行，不继承单点 Goto 的先对准策略。 */
  g_motion_control.large_yaw_align_enabled = 0U;
  g_motion_control.yaw_aligning = 0U;
  AdvanceMotion_SavePidPose(&g_motion.pose, g_motion.started_tick);
  g_motion_state = ADVANCE_MOTION_STATE_RUNNING;
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_RunState_t AdvanceMotion_FollowPathBlocking(const AdvanceMotion_PathPoint_t *points,
                                                           uint16_t point_count)
{
  if (AdvanceMotion_FollowPathEx(points, point_count) != ADVANCE_MOTION_STATUS_OK)
  {
    return ADVANCE_MOTION_STATE_CANCELED;
  }
  while (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    __WFI();
  }
  return g_motion_state;
}

/* 启动目标后仅等待 TIM6 将运动状态推进到终态。 */
AdvanceMotion_RunState_t AdvanceMotion_GotoGoalBlocking(const WorldGoalPose2D_t *goal, uint8_t acc)
{
  AdvanceMotion_Status_t status;

  status = AdvanceMotion_GotoPoseEx(goal, acc);
  if (status != ADVANCE_MOTION_STATUS_OK)
  {
    return ADVANCE_MOTION_STATE_CANCELED;
  }

  while (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    __WFI();
  }

  return g_motion_state;
}

AdvanceMotion_RunState_t AdvanceMotion_GotoPoseBlocking(float x_mm, float y_mm,
                                                         float yaw_deg, uint8_t acc)
{
  WorldGoalPose2D_t goal = {
      .x_mm = x_mm,
      .y_mm = y_mm,
      .yaw_deg = yaw_deg,
      .vmax_mm_s = ADVANCE_MOTION_DEFAULT_VMAX_MM_S,
      .wmax_deg_s = ADVANCE_MOTION_DEFAULT_WMAX_DEG_S,
      .timeout_ms = ADVANCE_MOTION_DEFAULT_TIMEOUT_MS,
      .goal_flags = ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW};

  return AdvanceMotion_GotoGoalBlocking(&goal, acc);
}

/* 周期性读取世界位姿，并推进 Goto 或连续路径控制状态机。 */
void AdvanceMotion_Update(void)
{
  uint32_t now_tick = HAL_GetTick();
  AdvanceMotion_Status_t pose_status;
  float vx_world_mm_s;
  float vy_world_mm_s;
  float wz_ccw_deg_s = 0.0f;
  float vmax_mm_s;
  float wmax_deg_s;
  float dt_s = 0.0f;
  float raw_linear_magnitude;
  float raw_wz_ccw_deg_s;
  float command_magnitude;
  uint32_t timeout_ms;
  uint8_t position_required;
  uint8_t position_control_enabled;
  uint8_t yaw_required;
  uint8_t path_final_stage = 0U;
  uint8_t linear_saturated;
  uint8_t yaw_saturated = 0U;

  AdvanceMotion_ApplyPendingPidConfig();
  AdvanceMotion_ApplyPendingPathControlConfig();

  if (g_motion_state != ADVANCE_MOTION_STATE_RUNNING)
  {
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }

  timeout_ms = (g_path.active != 0U) ? g_path.timeout_ms : g_motion.goal.timeout_ms;
  if ((timeout_ms > 0U) && ((now_tick - g_motion.started_tick) >= timeout_ms))
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_TIMEOUT);
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }

  if (g_path.active != 0U)
  {
    pose_status = AdvanceMotion_GetFreshPose(&g_motion.pose, now_tick);
  }
  else
  {
    position_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) != 0U) ? 1U : 0U;
    pose_status = (position_required != 0U)
                      ? AdvanceMotion_GetFreshPose(&g_motion.pose, now_tick)
                      : AdvanceMotion_GetFreshYaw(&g_motion.pose);
  }
  if (pose_status == ADVANCE_MOTION_STATUS_NO_ORIGIN)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_ORIGIN);
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }
  if (pose_status != ADVANCE_MOTION_STATUS_OK)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_POSE);
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }

  if ((g_motion_control.pid_history_valid != 0U) &&
      ((now_tick - g_motion_control.pid_last_tick) > 0U) &&
      ((now_tick - g_motion_control.pid_last_tick) <= ADVANCE_MOTION_PID_MAX_DT_MS))
  {
    dt_s = (float)(now_tick - g_motion_control.pid_last_tick) / 1000.0f;
  }
  AdvanceMotion_SavePidPose(&g_motion.pose, now_tick);

  if (g_path.active != 0U)
  {
    AdvanceMotion_UpdatePathReference(now_tick);
    path_final_stage = g_path.final_stage;
    if (path_final_stage == 0U)
    {
      g_motion_control.pid_integral_x_mm_s = 0.0f;
      g_motion_control.pid_integral_y_mm_s = 0.0f;
      g_motion_control.pid_integral_yaw_deg_s = 0.0f;
    }
  }

  /* 路径参考更新后再读取约束，保证本周期 PID 与当前参考一致。 */
  position_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) != 0U) ? 1U : 0U;
  yaw_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U) ? 1U : 0U;

  if (position_required != 0U)
  {
    g_motion.error_x_mm = g_motion.goal.x_mm - g_motion.pose.x_mm;
    g_motion.error_y_mm = g_motion.goal.y_mm - g_motion.pose.y_mm;
    g_motion.position_error_mm = sqrtf((g_motion.error_x_mm * g_motion.error_x_mm) +
                                       (g_motion.error_y_mm * g_motion.error_y_mm));
  }
  else
  {
    g_motion.error_x_mm = 0.0f;
    g_motion.error_y_mm = 0.0f;
    g_motion.position_error_mm = 0.0f;
  }
  g_motion.yaw_error_deg = yaw_required ? AdvanceWorld_WrapAngleDeg(g_motion.goal.yaw_deg - g_motion.pose.yaw_deg) : 0.0f;

  if ((g_motion_control.large_yaw_align_enabled != 0U) &&
      (position_required != 0U) && (yaw_required != 0U))
  {
    if ((g_motion_control.yaw_aligning != 0U) &&
        (AdvanceMotion_AbsFloat(g_motion.yaw_error_deg) <= ADVANCE_MOTION_LARGE_YAW_ALIGN_EXIT_DEG))
    {
      g_motion_control.yaw_aligning = 0U;
      AdvanceMotion_ResetLinearPid();
    }
    else if ((g_motion_control.yaw_aligning == 0U) &&
             (AdvanceMotion_AbsFloat(g_motion.yaw_error_deg) >= ADVANCE_MOTION_LARGE_YAW_ALIGN_ENTER_DEG))
    {
      g_motion_control.yaw_aligning = 1U;
      AdvanceMotion_ResetLinearPid();
    }
  }
  position_control_enabled = ((position_required != 0U) &&
                              (g_motion_control.yaw_aligning == 0U)) ? 1U : 0U;

  /* 只有在没有处于减速判定，或完全到达，或真正超时等终端状态下，才建议完全清除 PID 和进展。
   * 此处我们引入 Hysteresis (滞回/缓冲) 和 判定周期内微分保持(减速阻尼活性维持)。
   * 如果进入了容差并且正在平滑刹车，我们保留阻尼，不直接清空 PID 历史。
   */
  float dynamic_pos_tolerance = ADVANCE_MOTION_POS_TOLERANCE_MM;
  float dynamic_yaw_tolerance = ADVANCE_MOTION_YAW_TOLERANCE_DEG;

  /* 如果此前已经在减速判定中(arrive_hold_start_tick > 0 且尚未生成末端 arrived)，
   * 采用更宽的滞回边界(退出门限)，防止因为惯性微小抖出 10mm/1.5° 门限导致比例项 P 重踢(chattering)
   */
  if (g_motion_control.arrive_hold_start_tick != 0U)
  {
    dynamic_pos_tolerance = ADVANCE_MOTION_POS_TOLERANCE_MM + 6.0f;     /* 16.0mm 滞回退出边界 */
    dynamic_yaw_tolerance = ADVANCE_MOTION_YAW_TOLERANCE_DEG + 0.8f;   /* 2.3° 滞回退出边界 */
  }

  if (((g_path.active == 0U) || (path_final_stage != 0U)) &&
      ((position_required == 0U) || (g_motion.position_error_mm <= dynamic_pos_tolerance)) &&
      ((yaw_required == 0U) || (AdvanceMotion_AbsFloat(g_motion.yaw_error_deg) <= dynamic_yaw_tolerance)))
  {
    if (g_motion_control.arrival_stop_sent == 0U)
    {
      /* 保持判定期间已不再输出上一周期的非零速度。 */
      g_motion_control.arrival_stop_sent = Chassis_SmoothStop(g_motion_control.acc);
      if (g_motion_control.arrival_stop_sent == 0U)
      {
        AdvanceMotion_UpdateDebugSnapshot(now_tick,
                                          0U);
        return;
      }
    }
    if (g_motion_control.arrive_hold_start_tick == 0U)
    {
      g_motion_control.arrive_hold_start_tick = now_tick;
    }
    if ((now_tick - g_motion_control.arrive_hold_start_tick) >= ADVANCE_MOTION_ARRIVE_HOLD_MS)
    {
      /* 先保持到达容差，再发送硬停止，清除平滑减速后的残余漂移。 */
      if (g_motion_control.arrival_hard_stop_sent == 0U)
      {
        if (Chassis_Stop() == 0U)
        {
          AdvanceMotion_UpdateDebugSnapshot(now_tick,
                                            0U);
          return;
        }
        g_motion_control.arrival_hard_stop_sent = 1U;
      }
      AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_ARRIVED);
    }
    else
    {
      /* 🌟 核心优化点：在 150ms 减速滑行等待期间内，不暴力调用 ResetPidAndProgress!
       * 此时电机被强制处于低速度，但我们保留 pid_history_valid 与 measured_velocity，
       * 以免中途由于物理扰动跌出容差边界后瞬间产生无微分阻尼的大输出。
       */
      /* 暂时仅对位置积分项进行清零，微分和历史状态得以保留，以确保物理刹车的过冲阻尼性 */
      g_motion_control.pid_integral_x_mm_s = 0.0f;
      g_motion_control.pid_integral_y_mm_s = 0.0f;
      g_motion_control.pid_integral_yaw_deg_s = 0.0f;
    }
    AdvanceMotion_UpdateDebugSnapshot(now_tick,
                                      0U);
    return;
  }
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  g_motion_control.arrival_hard_stop_sent = 0U;

  vx_world_mm_s = 0.0f;
  vy_world_mm_s = 0.0f;
  raw_linear_magnitude = 0.0f;
  linear_saturated = 0U;
  if (position_control_enabled != 0U)
  {
    if ((g_path.active != 0U) && (path_final_stage == 0U))
    {
      const AdvanceMotion_PathPoint_t *a = &g_path.points[g_path.nearest_index];
      const AdvanceMotion_PathPoint_t *b = &g_path.points[g_path.nearest_index + 1U];
      float tx = b->x_mm - a->x_mm;
      float ty = b->y_mm - a->y_mm;
      float length = sqrtf((tx * tx) + (ty * ty));
      float normal_velocity;
      float normal_accel_ff_mm_s2;
      float normal_velocity_ff_mm_s;
      float normal_feedback_mm_s;
      float normal_command_mm_s;

      tx /= length;
      ty /= length;
      normal_velocity = (-ty * g_motion_control.measured_vx_world_mm_s) +
                        (tx * g_motion_control.measured_vy_world_mm_s);
      normal_accel_ff_mm_s2 =
          (g_path.reference_speed_mm_s * g_path.reference_speed_mm_s) *
          g_path.signed_curvature_1_mm;
      normal_accel_ff_mm_s2 = AdvanceWorld_LimitFloat(
          normal_accel_ff_mm_s2,
          -g_path_config_active.max_lateral_accel_mm_s2,
          g_path_config_active.max_lateral_accel_mm_s2);
      normal_velocity_ff_mm_s =
          g_path_config_active.curvature_ff_time_s * normal_accel_ff_mm_s2;
      /* cross_track 与 normal_velocity 正值表示左侧，normal_command 正值施加右法向；
       * 左弯的正曲率内侧在左方，因此曲率前馈使用负号。 */
      normal_feedback_mm_s =
          (g_path_config_active.kp_cross_track * g_path.cross_track_mm) +
          (g_path_config_active.kd_cross_track_velocity * normal_velocity);
      normal_command_mm_s = normal_feedback_mm_s - normal_velocity_ff_mm_s;
      g_path.measured_normal_velocity_mm_s = normal_velocity;
      g_path.normal_velocity_ff_mm_s = normal_velocity_ff_mm_s;
      g_path.normal_feedback_mm_s = normal_feedback_mm_s;
      vx_world_mm_s = g_path.feedforward_vx_mm_s + (ty * normal_command_mm_s);
      vy_world_mm_s = g_path.feedforward_vy_mm_s - (tx * normal_command_mm_s);
      vmax_mm_s = g_path.reference_speed_mm_s;
      raw_linear_magnitude = sqrtf((vx_world_mm_s * vx_world_mm_s) +
                                    (vy_world_mm_s * vy_world_mm_s));
      if (vmax_mm_s > 0.0f)
      {
        (void)AdvanceMotion_LimitVector(&vx_world_mm_s, &vy_world_mm_s, vmax_mm_s);
      }
      else
      {
        vx_world_mm_s = 0.0f;
        vy_world_mm_s = 0.0f;
      }
      linear_saturated = (raw_linear_magnitude > vmax_mm_s) ? 1U : 0U;
    }
    else
    {
      vx_world_mm_s = (g_pid_active.kp_pos * g_motion.error_x_mm) +
                      (g_pid_active.ki_pos * g_motion_control.pid_integral_x_mm_s) -
                      (g_pid_active.kd_pos * g_motion_control.measured_vx_world_mm_s);
      vy_world_mm_s = (g_pid_active.kp_pos * g_motion.error_y_mm) +
                      (g_pid_active.ki_pos * g_motion_control.pid_integral_y_mm_s) -
                      (g_pid_active.kd_pos * g_motion_control.measured_vy_world_mm_s);
      vmax_mm_s = AdvanceMotion_GetGoalVmax(&g_motion.goal);
      if ((g_motion_control.large_yaw_align_enabled != 0U) && (yaw_required != 0U))
      {
        vmax_mm_s *= AdvanceMotion_GetLargeYawAlignLinearScale(g_motion.yaw_error_deg);
      }
      raw_linear_magnitude = sqrtf((vx_world_mm_s * vx_world_mm_s) +
                                    (vy_world_mm_s * vy_world_mm_s));
      (void)AdvanceMotion_LimitVector(&vx_world_mm_s, &vy_world_mm_s, vmax_mm_s);
      linear_saturated = (raw_linear_magnitude > vmax_mm_s) ? 1U : 0U;
    }
  }

  if (yaw_required != 0U)
  {
    wmax_deg_s = AdvanceMotion_GetGoalWmax(&g_motion.goal);
    if ((g_path.active != 0U) && (path_final_stage == 0U))
    {
      raw_wz_ccw_deg_s = g_path.feedforward_wz_deg_s +
                         (g_path_config_active.kp_yaw * g_motion.yaw_error_deg) -
                         (g_path_config_active.kd_yaw_rate * g_motion_control.measured_wz_deg_s);
    }
    else
    {
      raw_wz_ccw_deg_s = (g_pid_active.kp_yaw * g_motion.yaw_error_deg) +
                          (g_pid_active.ki_yaw * g_motion_control.pid_integral_yaw_deg_s) -
                          (g_pid_active.kd_yaw * g_motion_control.measured_wz_deg_s);
    }
    wz_ccw_deg_s = AdvanceWorld_LimitFloat(
        raw_wz_ccw_deg_s,
        -wmax_deg_s,
        wmax_deg_s);
    yaw_saturated = (AdvanceMotion_AbsFloat(raw_wz_ccw_deg_s) > wmax_deg_s) ? 1U : 0U;
  }
  g_path.command_wz_deg_s = wz_ccw_deg_s;

  AdvanceMotion_UpdatePidIntegral(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, dt_s,
                                    linear_saturated, yaw_saturated,
                                    ((g_path.active == 0U) || (path_final_stage != 0U))
                                        ? position_control_enabled
                                        : 0U,
                                    ((g_path.active == 0U) || (path_final_stage != 0U))
                                        ? yaw_required
                                        : 0U);
  command_magnitude = sqrtf((vx_world_mm_s * vx_world_mm_s) +
                             (vy_world_mm_s * vy_world_mm_s));
  if ((position_control_enabled != 0U) && (AdvanceMotion_HasNoProgress(now_tick, command_magnitude) != 0U))
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_CANCELED);
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }

  if (((position_required != 0U) &&
       (AdvanceMotion_ApplyWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s,
                                            g_motion_control.acc, &g_motion.pose) == ADVANCE_MOTION_STATUS_OK)) ||
      ((position_required == 0U) &&
       (Chassis_SetBodyVelocityEx(0.0f, 0.0f, wz_ccw_deg_s, g_motion_control.acc) != 0U)))
  {
    g_motion_control.command_vx_world_mm_s = vx_world_mm_s;
    g_motion_control.command_vy_world_mm_s = vy_world_mm_s;
    g_motion_control.command_wz_ccw_deg_s = wz_ccw_deg_s;
  }
  g_motion.updated_tick = now_tick;
  AdvanceMotion_UpdateDebugSnapshot(now_tick,
                                    ((linear_saturated != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_LINEAR_SATURATED : 0U) |
                                         ((yaw_saturated != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_SATURATED : 0U) |
                                         ((g_motion_control.yaw_aligning != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_ALIGNING : 0U));
}

void AdvanceMotion_ResetYawControl(void)
{
  AdvanceMotion_ClearYawControlTerms();
  g_motion_control.arrive_hold_start_tick = 0U;
}

/* 取消当前运动任务、释放控制权并停止底盘。 */
void AdvanceMotion_Cancel(void)
{
  AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_CANCELED);
}

/* 仅在存在运行中任务时取消运动。 */
void AdvanceMotion_CancelIfActive(void)
{
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    AdvanceMotion_Cancel();
  }
}

/* 读取当前运动状态、目标位姿和误差。 */
AdvanceMotion_Status_t AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status)
{
  uint32_t primask;

  if (status == 0)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *status = (AdvanceMotion_RuntimeStatus_t){
      g_motion_state,
      g_motion.goal,
      g_motion.pose,
      g_motion.error_x_mm,
      g_motion.error_y_mm,
      g_motion.position_error_mm,
      g_motion.yaw_error_deg,
      g_motion.started_tick,
      g_motion.updated_tick};
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_GetDebugSnapshot(AdvanceMotion_DebugSnapshot_t *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = g_motion_debug;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_GetPidConfig(AdvanceMotion_PidConfig_t *config,
                                                   uint32_t *revision)
{
  uint32_t primask;

  if ((config == NULL) || (revision == NULL))
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *config = g_pid_active;
  *revision = g_pid_active_revision;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_RequestPidConfig(const AdvanceMotion_PidConfig_t *config,
                                                       uint32_t *revision)
{
  uint32_t primask;

  if ((revision == NULL) || (AdvanceMotion_IsPidConfigValid(config) == 0U))
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  ++g_pid_next_revision;
  if (g_pid_next_revision == 0U)
  {
    ++g_pid_next_revision;
  }
  g_pid_pending = *config;
  g_pid_pending_revision = g_pid_next_revision;
  g_pid_pending_valid = 1U;
  *revision = g_pid_pending_revision;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_RestoreDefaultPid(uint32_t *revision)
{
  return AdvanceMotion_RequestPidConfig(&g_pid_default, revision);
}

AdvanceMotion_Status_t AdvanceMotion_GetPathControlConfig(
    AdvanceMotion_PathControlConfig_t *config, uint32_t *revision)
{
  uint32_t primask;

  if ((config == NULL) || (revision == NULL))
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  *config = g_path_config_active;
  *revision = g_path_config_active_revision;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_RequestPathControlConfig(
    const AdvanceMotion_PathControlConfig_t *config, uint32_t *revision)
{
  uint32_t primask;

  if ((revision == NULL) || (AdvanceMotion_IsPathControlConfigValid(config) == 0U))
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  ++g_path_config_next_revision;
  if (g_path_config_next_revision == 0U)
  {
    ++g_path_config_next_revision;
  }
  g_path_config_pending = *config;
  g_path_config_pending_revision = g_path_config_next_revision;
  g_path_config_pending_valid = 1U;
  *revision = g_path_config_pending_revision;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_RestoreDefaultPathControl(uint32_t *revision)
{
  return AdvanceMotion_RequestPathControlConfig(&g_path_config_default, revision);
}

AdvanceMotion_Status_t AdvanceMotion_SetLargeYawAlignEnabled(uint8_t enabled)
{
  if (enabled > 1U)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }
  g_large_yaw_align_enabled = enabled;
  return ADVANCE_MOTION_STATUS_OK;
}

AdvanceMotion_Status_t AdvanceMotion_GetLargeYawAlignEnabled(uint8_t *enabled)
{
  if (enabled == NULL)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  *enabled = g_large_yaw_align_enabled;
  return ADVANCE_MOTION_STATUS_OK;
}
