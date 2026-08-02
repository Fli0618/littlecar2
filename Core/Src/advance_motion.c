#include "advance_motion.h"

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
  uint8_t terminal_stop_pending;
  uint8_t pid_history_valid;
  uint8_t large_yaw_align_enabled;
  uint8_t yaw_aligning;
  uint8_t acc;
} AdvanceMotion_Control_t;

typedef struct
{
  const WorldGoalPose2D_t *points;
  uint16_t point_count;
  uint16_t nearest_index;
  uint16_t target_index;
  uint16_t progress_index;
  uint32_t progress_tick;
  uint8_t active;
} AdvanceMotion_PathContext_t;

static AdvanceMotion_RuntimeStatus_t g_motion = {ADVANCE_MOTION_STATE_IDLE};
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
static uint8_t g_large_yaw_align_enabled;

static const AdvanceMotion_PidConfig_t g_pid_default = {
    ADVANCE_MOTION_DEFAULT_KP_POS,
    ADVANCE_MOTION_DEFAULT_KI_POS,
    ADVANCE_MOTION_DEFAULT_KD_POS,
    ADVANCE_MOTION_DEFAULT_KP_YAW,
    ADVANCE_MOTION_DEFAULT_KI_YAW,
    ADVANCE_MOTION_DEFAULT_KD_YAW};

/* 返回浮点数的绝对值。 */
static float AdvanceMotion_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void AdvanceMotion_ClearPathContext(void)
{
  g_path = (AdvanceMotion_PathContext_t){0};
}

/* 清除仅属于一轮 GotoPose 的 PID 与外部进展校验历史。 */
static void AdvanceMotion_ResetPidAndProgress(void)
{
  g_motion_control.pid_last_tick = 0U;
  g_motion_control.no_progress_start_tick = 0U;
  g_motion_control.last_pose_updated_tick = 0U;
  g_motion_control.last_yaw_updated_tick = 0U;
  g_motion_control.pid_integral_x_mm_s = 0.0f;
  g_motion_control.pid_integral_y_mm_s = 0.0f;
  g_motion_control.pid_integral_yaw_deg_s = 0.0f;
  g_motion_control.pid_last_x_mm = 0.0f;
  g_motion_control.pid_last_y_mm = 0.0f;
  g_motion_control.pid_last_yaw_deg = 0.0f;
  g_motion_control.measured_vx_world_mm_s = 0.0f;
  g_motion_control.measured_vy_world_mm_s = 0.0f;
  g_motion_control.measured_wz_deg_s = 0.0f;
  g_motion_control.command_vx_world_mm_s = 0.0f;
  g_motion_control.command_vy_world_mm_s = 0.0f;
  g_motion_control.command_wz_ccw_deg_s = 0.0f;
  g_motion_control.no_progress_reference_error_mm = 0.0f;
  g_motion_control.pid_history_valid = 0U;
  g_motion_control.yaw_aligning = 0U;
}

/* Reset the linear loop when crossing between alignment and coupled motion. */
static void AdvanceMotion_ResetLinearPid(void)
{
  g_motion_control.no_progress_start_tick = 0U;
  g_motion_control.pid_integral_x_mm_s = 0.0f;
  g_motion_control.pid_integral_y_mm_s = 0.0f;
  g_motion_control.pid_last_x_mm = 0.0f;
  g_motion_control.pid_last_y_mm = 0.0f;
  g_motion_control.measured_vx_world_mm_s = 0.0f;
  g_motion_control.measured_vy_world_mm_s = 0.0f;
  g_motion_control.pid_history_valid = 0U;
}

static float AdvanceMotion_GetLargeYawAlignLinearScale(float yaw_error_deg)
{
  float ratio = AdvanceMotion_AbsFloat(yaw_error_deg) /
                ADVANCE_MOTION_LARGE_YAW_ALIGN_ENTER_DEG;

  ratio = AdvanceWorld_LimitFloat(ratio, 0.0f, 1.0f);
  return 1.0f - ((1.0f - ADVANCE_MOTION_LARGE_YAW_ALIGN_LINEAR_MIN_SCALE) * ratio);
}

static void AdvanceMotion_UpdateDebugSnapshot(uint32_t now_tick, uint8_t flags)
{
  if (g_path.active != 0U)
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_PATH_ACTIVE;
  }
  if (AdvanceWorld_GetYawSource() == ADVANCE_WORLD_YAW_SOURCE_OPS)
  {
    flags |= ADVANCE_MOTION_DEBUG_FLAG_YAW_SOURCE_OPS;
  }
  g_motion_debug.tick = now_tick;
  g_motion_debug.pid_revision = g_pid_active_revision;
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

/* 以位置误差的下降量校验外部闭环是否仍在取得进展。 */
static uint8_t AdvanceMotion_HasNoProgress(uint32_t now_tick, float command_magnitude)
{
  if (g_path.active != 0U)
  {
    if (command_magnitude < ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S)
    {
      return 0U;
    }
    if (g_path.nearest_index > g_path.progress_index)
    {
      g_path.progress_index = g_path.nearest_index;
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

static uint16_t AdvanceMotion_FindNearestPathPoint(const WorldPose2D_t *pose,
                                                    uint16_t start_index)
{
  uint16_t end_index;
  uint16_t index;
  uint16_t nearest_index;
  float dx;
  float dy;
  float distance_squared;
  float nearest_distance_squared;

  if ((pose == NULL) || (g_path.points == NULL) || (g_path.point_count == 0U))
  {
    return 0U;
  }
  if (start_index >= g_path.point_count)
  {
    start_index = (uint16_t)(g_path.point_count - 1U);
  }
  end_index = (uint16_t)(start_index + (ADVANCE_MOTION_PATH_SEARCH_POINTS - 1U));
  if ((end_index < start_index) || (end_index >= g_path.point_count))
  {
    end_index = (uint16_t)(g_path.point_count - 1U);
  }

  nearest_index = start_index;
  dx = g_path.points[start_index].x_mm - pose->x_mm;
  dy = g_path.points[start_index].y_mm - pose->y_mm;
  nearest_distance_squared = (dx * dx) + (dy * dy);
  for (index = (uint16_t)(start_index + 1U); index <= end_index; ++index)
  {
    dx = g_path.points[index].x_mm - pose->x_mm;
    dy = g_path.points[index].y_mm - pose->y_mm;
    distance_squared = (dx * dx) + (dy * dy);
    if (distance_squared < nearest_distance_squared)
    {
      nearest_distance_squared = distance_squared;
      nearest_index = index;
    }
  }

  return nearest_index;
}

static uint16_t AdvanceMotion_FindLookaheadPoint(uint16_t nearest_index)
{
  uint16_t index;
  float accumulated_mm = 0.0f;
  float dx;
  float dy;
  float segment_mm;

  if ((g_path.points == NULL) || (g_path.point_count == 0U))
  {
    return 0U;
  }
  if (nearest_index >= (uint16_t)(g_path.point_count - 1U))
  {
    return (uint16_t)(g_path.point_count - 1U);
  }

  for (index = (uint16_t)(nearest_index + 1U); index < g_path.point_count; ++index)
  {
    dx = g_path.points[index].x_mm - g_path.points[index - 1U].x_mm;
    dy = g_path.points[index].y_mm - g_path.points[index - 1U].y_mm;
    segment_mm = sqrtf((dx * dx) + (dy * dy));
    accumulated_mm += segment_mm;
    if (accumulated_mm >= ADVANCE_MOTION_PATH_LOOKAHEAD_MM)
    {
      return index;
    }
  }

  return (uint16_t)(g_path.point_count - 1U);
}

static void AdvanceMotion_UpdatePathReference(void)
{
  uint16_t index;
  uint16_t new_target_index;
  float accumulated_mm = 0.0f;
  float dx;
  float dy;
  float segment_mm;
  float remaining_mm;
  float ratio;
  const WorldGoalPose2D_t *from;
  const WorldGoalPose2D_t *to;

  g_path.nearest_index = AdvanceMotion_FindNearestPathPoint(&g_motion.pose, g_path.nearest_index);
  new_target_index = AdvanceMotion_FindLookaheadPoint(g_path.nearest_index);
  if ((new_target_index == (uint16_t)(g_path.point_count - 1U)) &&
      (g_path.target_index != new_target_index))
  {
    g_motion_control.pid_integral_x_mm_s = 0.0f;
    g_motion_control.pid_integral_y_mm_s = 0.0f;
    g_motion_control.pid_integral_yaw_deg_s = 0.0f;
  }
  g_path.target_index = new_target_index;
  if (new_target_index == (uint16_t)(g_path.point_count - 1U))
  {
    g_motion.goal = g_path.points[new_target_index];
    return;
  }

  for (index = (uint16_t)(g_path.nearest_index + 1U); index <= new_target_index; ++index)
  {
    from = &g_path.points[index - 1U];
    to = &g_path.points[index];
    dx = to->x_mm - from->x_mm;
    dy = to->y_mm - from->y_mm;
    segment_mm = sqrtf((dx * dx) + (dy * dy));
    if ((segment_mm > 0.0f) && ((accumulated_mm + segment_mm) >= ADVANCE_MOTION_PATH_LOOKAHEAD_MM))
    {
      remaining_mm = ADVANCE_MOTION_PATH_LOOKAHEAD_MM - accumulated_mm;
      ratio = AdvanceWorld_LimitFloat(remaining_mm / segment_mm, 0.0f, 1.0f);
      g_motion.goal.x_mm = from->x_mm + (ratio * dx);
      g_motion.goal.y_mm = from->y_mm + (ratio * dy);
      g_motion.goal.yaw_deg = AdvanceWorld_WrapAngleDeg(
          from->yaw_deg + (ratio * AdvanceWorld_WrapAngleDeg(to->yaw_deg - from->yaw_deg)));
      g_motion.goal.vmax_mm_s = from->vmax_mm_s + (ratio * (to->vmax_mm_s - from->vmax_mm_s));
      g_motion.goal.wmax_deg_s = from->wmax_deg_s + (ratio * (to->wmax_deg_s - from->wmax_deg_s));
      g_motion.goal.timeout_ms = g_path.points[g_path.point_count - 1U].timeout_ms;
      g_motion.goal.goal_flags = to->goal_flags;
      return;
    }
    accumulated_mm += segment_mm;
  }

  g_motion.goal = g_path.points[new_target_index];
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
static AdvanceMotion_Status_t AdvanceMotion_GetFreshPose(WorldPose2D_t *pose)
{
  AdvanceWorld_Status_t world_status;

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

  if ((HAL_GetTick() - pose->updated_tick) > ADVANCE_MOTION_POSE_TIMEOUT_MS)
  {
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }

  return ADVANCE_MOTION_STATUS_OK;
}

static void AdvanceMotion_UpdateInactiveDebugSnapshot(uint32_t now_tick)
{
  uint8_t flags = 0U;

  if (AdvanceMotion_GetFreshPose(&g_motion.pose) == ADVANCE_MOTION_STATUS_OK)
  {
    flags = ADVANCE_MOTION_DEBUG_FLAG_VALID |
            ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH;
    if ((now_tick - g_motion.pose.yaw_updated_tick) <= ADVANCE_MOTION_YAW_TIMEOUT_MS)
    {
      flags |= ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH;
    }
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
    pose_status = AdvanceMotion_GetFreshPose(&current_pose);
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
  g_motion = (AdvanceMotion_RuntimeStatus_t){ADVANCE_MOTION_STATE_IDLE};
  g_motion_control = (AdvanceMotion_Control_t){0};
  AdvanceMotion_ClearPathContext();
  g_motion_state = ADVANCE_MOTION_STATE_IDLE;
  g_pid_active = g_pid_default;
  g_pid_pending = g_pid_default;
  g_pid_pending_valid = 0U;
  g_pid_active_revision = 0U;
  g_pid_pending_revision = 0U;
  g_pid_next_revision = 0U;
  g_large_yaw_align_enabled = ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE;
  AdvanceMotion_UpdateDebugSnapshot(HAL_GetTick(), 0U);
}

/* 设置世界坐标系速度，并取消正在执行的到点任务。 */
AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    g_motion_state = ADVANCE_MOTION_STATE_CANCELED;
    g_motion.updated_tick = HAL_GetTick();
    g_motion_control.arrive_hold_start_tick = 0U;
    g_motion_control.arrival_stop_sent = 0U;
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
                    ? AdvanceMotion_GetFreshPose(&pose)
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

AdvanceMotion_Status_t AdvanceMotion_FollowPathEx(const WorldGoalPose2D_t *points,
                                                   uint16_t point_count, uint8_t acc)
{
  WorldPose2D_t pose;
  AdvanceMotion_Status_t pose_status;
  uint16_t index;

  if ((points == NULL) || (point_count < 2U))
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }
  for (index = 0U; index < point_count; ++index)
  {
    if ((AdvanceMotion_IsGoalValid(&points[index]) == 0U) ||
        ((points[index].goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) == 0U))
    {
      return ADVANCE_MOTION_STATUS_INVALID_PARAM;
    }
  }
  if (g_motion_state == ADVANCE_MOTION_STATE_RUNNING)
  {
    return ADVANCE_MOTION_STATUS_BUSY;
  }

  pose_status = AdvanceMotion_GetFreshPose(&pose);
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
  g_path.active = 1U;
  g_motion.goal = points[0];
  g_motion.pose = pose;
  g_motion.started_tick = g_path.progress_tick;
  g_motion.updated_tick = g_motion.started_tick;
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  AdvanceMotion_ResetPidAndProgress();
  AdvanceMotion_UpdatePathReference();
  g_motion.error_x_mm = 0.0f;
  g_motion.error_y_mm = 0.0f;
  g_motion.position_error_mm = 0.0f;
  g_motion.yaw_error_deg = 0.0f;
  g_motion_control.acc = acc;
  g_motion_control.large_yaw_align_enabled = g_large_yaw_align_enabled;
  g_motion_control.yaw_aligning =
      ((g_motion_control.large_yaw_align_enabled != 0U) &&
       ((g_motion.goal.goal_flags & (ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW)) ==
        (ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW)) &&
       (AdvanceMotion_AbsFloat(AdvanceWorld_WrapAngleDeg(g_motion.goal.yaw_deg - pose.yaw_deg)) >=
        ADVANCE_MOTION_LARGE_YAW_ALIGN_ENTER_DEG)) ? 1U : 0U;
  AdvanceMotion_SavePidPose(&g_motion.pose, g_motion.started_tick);
  g_motion_state = ADVANCE_MOTION_STATE_RUNNING;
  return ADVANCE_MOTION_STATUS_OK;
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

/* 周期性读取世界位姿，计算误差并驱动到点控制状态机。 */
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

  if (g_motion_state != ADVANCE_MOTION_STATE_RUNNING)
  {
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }

  timeout_ms = (g_path.active != 0U)
                   ? g_path.points[g_path.point_count - 1U].timeout_ms
                   : g_motion.goal.timeout_ms;
  if ((timeout_ms > 0U) && ((now_tick - g_motion.started_tick) >= timeout_ms))
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_TIMEOUT);
    AdvanceMotion_UpdateInactiveDebugSnapshot(now_tick);
    return;
  }

  position_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) != 0U) ? 1U : 0U;
  yaw_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U) ? 1U : 0U;
  pose_status = (position_required != 0U)
                    ? AdvanceMotion_GetFreshPose(&g_motion.pose)
                    : AdvanceMotion_GetFreshYaw(&g_motion.pose);
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

  if (g_path.active != 0U)
  {
    AdvanceMotion_UpdatePathReference();
    path_final_stage = (g_path.target_index == (uint16_t)(g_path.point_count - 1U)) ? 1U : 0U;
    if (path_final_stage == 0U)
    {
      g_motion_control.pid_integral_x_mm_s = 0.0f;
      g_motion_control.pid_integral_y_mm_s = 0.0f;
      g_motion_control.pid_integral_yaw_deg_s = 0.0f;
    }
  }

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

  if (((g_path.active == 0U) || (path_final_stage != 0U)) &&
      ((position_required == 0U) || (g_motion.position_error_mm <= ADVANCE_MOTION_POS_TOLERANCE_MM)) &&
      ((yaw_required == 0U) || (AdvanceMotion_AbsFloat(g_motion.yaw_error_deg) <= ADVANCE_MOTION_YAW_TOLERANCE_DEG)))
  {
    AdvanceMotion_ResetPidAndProgress();
    if (g_motion_control.arrival_stop_sent == 0U)
    {
      /* 保持判定期间已不再输出上一周期的非零速度。 */
      g_motion_control.arrival_stop_sent = Chassis_SmoothStop(g_motion_control.acc);
      if (g_motion_control.arrival_stop_sent == 0U)
      {
        AdvanceMotion_UpdateDebugSnapshot(now_tick,
                                          ADVANCE_MOTION_DEBUG_FLAG_VALID |
                                              ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH |
                                              ((yaw_required != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH : 0U));
        return;
      }
    }
    if (g_motion_control.arrive_hold_start_tick == 0U)
    {
      g_motion_control.arrive_hold_start_tick = now_tick;
    }
    if ((now_tick - g_motion_control.arrive_hold_start_tick) >= ADVANCE_MOTION_ARRIVE_HOLD_MS)
    {
      AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_ARRIVED);
    }
    AdvanceMotion_UpdateDebugSnapshot(now_tick,
                                      ADVANCE_MOTION_DEBUG_FLAG_VALID |
                                          ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH |
                                          ((yaw_required != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH : 0U));
    return;
  }
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;

  if ((g_motion_control.pid_history_valid != 0U) &&
      ((now_tick - g_motion_control.pid_last_tick) > 0U) &&
      ((now_tick - g_motion_control.pid_last_tick) <= ADVANCE_MOTION_PID_MAX_DT_MS))
  {
    dt_s = (float)(now_tick - g_motion_control.pid_last_tick) / 1000.0f;
  }

  AdvanceMotion_SavePidPose(&g_motion.pose, now_tick);

  vx_world_mm_s = 0.0f;
  vy_world_mm_s = 0.0f;
  raw_linear_magnitude = 0.0f;
  linear_saturated = 0U;
  if (position_control_enabled != 0U)
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

  if (yaw_required != 0U)
  {
    wmax_deg_s = AdvanceMotion_GetGoalWmax(&g_motion.goal);
    raw_wz_ccw_deg_s = (g_pid_active.kp_yaw * g_motion.yaw_error_deg) +
                        (g_pid_active.ki_yaw * g_motion_control.pid_integral_yaw_deg_s) -
                        (g_pid_active.kd_yaw * g_motion_control.measured_wz_deg_s);
    wz_ccw_deg_s = AdvanceWorld_LimitFloat(
        raw_wz_ccw_deg_s,
        -wmax_deg_s,
        wmax_deg_s);
    yaw_saturated = (AdvanceMotion_AbsFloat(raw_wz_ccw_deg_s) > wmax_deg_s) ? 1U : 0U;
  }

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
                                    ADVANCE_MOTION_DEBUG_FLAG_VALID |
                                        ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH |
                                         ((yaw_required != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH : 0U) |
                                         ((linear_saturated != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_LINEAR_SATURATED : 0U) |
                                         ((yaw_saturated != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_SATURATED : 0U) |
                                         ((g_motion_control.yaw_aligning != 0U) ? ADVANCE_MOTION_DEBUG_FLAG_YAW_ALIGNING : 0U));
}

void AdvanceMotion_ResetYawControl(void)
{
  g_motion_control.pid_integral_yaw_deg_s = 0.0f;
  g_motion_control.last_yaw_updated_tick = 0U;
  g_motion_control.pid_last_yaw_deg = 0.0f;
  g_motion_control.measured_wz_deg_s = 0.0f;
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.pid_history_valid = 0U;
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
  *status = g_motion;
  status->state = g_motion_state;
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
