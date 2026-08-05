#include "advance_holonomic_position.h"
#include "advance_holonomic_position_config.h"

#include "advance_control.h"
#include "advance_chassis.h"
#include "main.h"
#include <math.h>

/* 通用一维运动轮廓：平移与航向复用同一更新函数。 */
typedef struct
{
  float goal_position;
  float direction;
  float acceleration;
  float deceleration;
  float peak_velocity;
  float accel_time_s;
  float cruise_time_s;
  float decel_time_s;
  float total_time_s;
  float accel_distance;
  float cruise_distance;
  float elapsed_time_s;
  float position;
  float velocity;
  uint8_t finished;
} AdvanceHolonomic_Profile1D_t;

/* 统一参考状态：参考位姿 + 参考速度，未来曲线轨迹模块可直接复用。 */
typedef struct
{
  float x_mm;
  float y_mm;
  float yaw_deg;
  float vx_world_mm_s;
  float vy_world_mm_s;
  float wz_deg_s;
} AdvanceHolonomic_Reference_t;

/* 车体坐标系速度：+X 向右，+Y 向前，+WZ 俯视逆时针。 */
typedef struct
{
  float right_mm_s;
  float forward_mm_s;
  float wz_deg_s;
} AdvanceHolonomic_BodyVelocity_t;

typedef struct
{
  AdvanceHolonomic_RunState_t pending_terminal_state;
  WorldGoalPose2D_t goal;
  uint8_t position_required;
  uint8_t yaw_required;
  uint8_t acc;

  /* 平移标量路径：起点/终点/方向/长度，X/Y 同步完成保持直线 */
  float start_x_mm;
  float start_y_mm;
  float start_yaw_deg;
  float goal_x_mm;
  float goal_y_mm;
  float goal_yaw_deg;
  float direction_x;
  float direction_y;
  float path_length_mm;
  AdvanceHolonomic_Profile1D_t linear_profile;
  AdvanceHolonomic_Profile1D_t yaw_profile;

  /* 速度估计：滤波后的世界速度 + 最近一帧位姿 */
  float filtered_vx_world_mm_s;
  float filtered_vy_world_mm_s;
  float filtered_wz_deg_s;
  WorldPose2D_t last_pose;
  uint32_t last_pose_updated_tick;
  uint32_t last_yaw_updated_tick;
  uint8_t velocity_history_valid;
  AdvanceHolonomic_BodyVelocity_t measured;

  WorldPose2D_t latest_pose;
  uint32_t started_tick;
  uint32_t last_update_tick;
  uint32_t arrive_hold_start_tick;
  uint8_t stopping;

  AdvanceHolonomic_Config_t config;
  AdvanceHolonomic_DebugSnapshot_t debug;
} AdvanceHolonomic_Module_t;

static AdvanceHolonomic_Module_t g_holonomic = {
    .pending_terminal_state = ADVANCE_HOLONOMIC_STATE_IDLE};

/* 运行状态由 TIM6 中断写入、阻塞任务读取，独立声明为 volatile。 */
static volatile AdvanceHolonomic_RunState_t g_holonomic_state = ADVANCE_HOLONOMIC_STATE_IDLE;

static const AdvanceHolonomic_Config_t g_holonomic_default = {
    .linear_accel_mm_s2 = ADVANCE_HOLONOMIC_DEFAULT_LINEAR_ACCEL_MM_S2,
    .linear_decel_mm_s2 = ADVANCE_HOLONOMIC_DEFAULT_LINEAR_DECEL_MM_S2,
    .yaw_accel_deg_s2 = ADVANCE_HOLONOMIC_DEFAULT_YAW_ACCEL_DEG_S2,
    .kp_forward = ADVANCE_HOLONOMIC_DEFAULT_KP_FORWARD,
    .kv_forward = ADVANCE_HOLONOMIC_DEFAULT_KV_FORWARD,
    .kp_lateral = ADVANCE_HOLONOMIC_DEFAULT_KP_LATERAL,
    .kv_lateral = ADVANCE_HOLONOMIC_DEFAULT_KV_LATERAL,
    .kp_yaw = ADVANCE_HOLONOMIC_DEFAULT_KP_YAW,
    .kv_yaw = ADVANCE_HOLONOMIC_DEFAULT_KV_YAW,
    .forward_scale = ADVANCE_HOLONOMIC_DEFAULT_FORWARD_SCALE,
    .lateral_scale = ADVANCE_HOLONOMIC_DEFAULT_LATERAL_SCALE,
    .yaw_scale = ADVANCE_HOLONOMIC_DEFAULT_YAW_SCALE};

/* 解析梯形/三角形速度轮廓：启动时预计算分段参数，运行时按累计时间解析求值。 */
static void AdvanceHolonomic_Profile1D_Init(AdvanceHolonomic_Profile1D_t *profile,
                                            float goal_position,
                                            float max_velocity,
                                            float acceleration,
                                            float deceleration)
{
  float distance;
  float accel_distance_at_max;
  float decel_distance_at_max;

  if (profile == NULL)
  {
    return;
  }

  profile->goal_position = goal_position;
  profile->direction = (goal_position >= 0.0f) ? 1.0f : -1.0f;
  profile->acceleration = acceleration;
  profile->deceleration = deceleration;
  profile->elapsed_time_s = 0.0f;
  profile->position = 0.0f;
  profile->velocity = 0.0f;
  profile->finished = 0U;

  distance = fabsf(goal_position);
  if (distance <= 0.0001f)
  {
    /* 零距离目标直接完成 */
    profile->peak_velocity = 0.0f;
    profile->accel_time_s = 0.0f;
    profile->cruise_time_s = 0.0f;
    profile->decel_time_s = 0.0f;
    profile->total_time_s = 0.0f;
    profile->accel_distance = 0.0f;
    profile->cruise_distance = 0.0f;
    profile->position = goal_position;
    profile->velocity = 0.0f;
    profile->finished = 1U;
    return;
  }

  accel_distance_at_max = (max_velocity * max_velocity) / (2.0f * acceleration);
  decel_distance_at_max = (max_velocity * max_velocity) / (2.0f * deceleration);

  if ((accel_distance_at_max + decel_distance_at_max) <= distance)
  {
    /* 梯形轮廓：可达最大速度并存在巡航段 */
    profile->peak_velocity = max_velocity;
    profile->accel_distance =
        (profile->peak_velocity * profile->peak_velocity) / (2.0f * acceleration);
    profile->cruise_distance =
        distance - profile->accel_distance -
        ((profile->peak_velocity * profile->peak_velocity) / (2.0f * deceleration));
    if (profile->cruise_distance < 0.0f)
    {
      /* 浮点误差防护，防止出现极小负巡航距离 */
      profile->cruise_distance = 0.0f;
    }
    profile->accel_time_s = profile->peak_velocity / acceleration;
    profile->cruise_time_s = profile->cruise_distance / profile->peak_velocity;
    profile->decel_time_s = profile->peak_velocity / deceleration;
  }
  else
  {
    /* 三角形轮廓：无法达到最大速度 */
    profile->peak_velocity =
        sqrtf((2.0f * distance * acceleration * deceleration) /
              (acceleration + deceleration));
    profile->accel_distance =
        (profile->peak_velocity * profile->peak_velocity) / (2.0f * acceleration);
    profile->cruise_distance = 0.0f;
    profile->accel_time_s = profile->peak_velocity / acceleration;
    profile->cruise_time_s = 0.0f;
    profile->decel_time_s = profile->peak_velocity / deceleration;
  }

  profile->total_time_s =
      profile->accel_time_s + profile->cruise_time_s + profile->decel_time_s;
}

/* 解析求值：加速/巡航/减速分段由累计时间确定，最终参考速度精确归零。 */
static void AdvanceHolonomic_Profile1D_Update(AdvanceHolonomic_Profile1D_t *profile, float dt_s)
{
  float distance;
  float t;
  float position_abs;
  float velocity_abs;

  if ((profile == NULL) || (dt_s <= 0.0f) || (profile->finished != 0U))
  {
    return;
  }

  profile->elapsed_time_s += dt_s;
  distance = fabsf(profile->goal_position);

  if (profile->elapsed_time_s >= profile->total_time_s)
  {
    /* 轮廓结束：位置精确等于目标，速度精确归零 */
    profile->position = profile->goal_position;
    profile->velocity = 0.0f;
    profile->finished = 1U;
    return;
  }

  if (profile->elapsed_time_s <= profile->accel_time_s)
  {
    /* 加速段：position = 0.5*a*t^2，velocity = a*t */
    t = profile->elapsed_time_s;
    position_abs = 0.5f * profile->acceleration * t * t;
    velocity_abs = profile->acceleration * t;
  }
  else if (profile->elapsed_time_s <= (profile->accel_time_s + profile->cruise_time_s))
  {
    /* 巡航段：位置线性推进，速度保持峰值 */
    t = profile->elapsed_time_s - profile->accel_time_s;
    position_abs = profile->accel_distance + (profile->peak_velocity * t);
    velocity_abs = profile->peak_velocity;
  }
  else
  {
    /* 减速段：position = accel + cruise + peak*t - 0.5*d*t^2 */
    t = profile->elapsed_time_s - profile->accel_time_s - profile->cruise_time_s;
    position_abs = profile->accel_distance + profile->cruise_distance +
                   (profile->peak_velocity * t) -
                   (0.5f * profile->deceleration * t * t);
    velocity_abs = profile->peak_velocity - (profile->deceleration * t);
  }

  /* 末端浮点误差保护：速度非负、位置不越界 */
  if (velocity_abs < 0.0f)
  {
    velocity_abs = 0.0f;
  }
  if (position_abs < 0.0f)
  {
    position_abs = 0.0f;
  }
  else if (position_abs > distance)
  {
    position_abs = distance;
  }

  profile->position = profile->direction * position_abs;
  profile->velocity = profile->direction * velocity_abs;
}

/* 目标校验：NULL、非有限浮点、边界、速度上限、超时与标志位。 */
static uint8_t AdvanceHolonomic_IsGoalValid(const WorldGoalPose2D_t *goal)
{
  if ((goal == NULL) ||
      (isfinite(goal->x_mm) == 0) ||
      (isfinite(goal->y_mm) == 0) ||
      (isfinite(goal->yaw_deg) == 0) ||
      (isfinite(goal->vmax_mm_s) == 0) ||
      (isfinite(goal->wmax_deg_s) == 0))
  {
    return 0U;
  }

  if ((((goal->goal_flags & ADVANCE_HOLONOMIC_GOAL_USE_POSITION) != 0U) &&
       ((goal->x_mm < ADVANCE_HOLONOMIC_WORLD_X_MIN_MM) ||
        (goal->x_mm > ADVANCE_HOLONOMIC_WORLD_X_MAX_MM) ||
        (goal->y_mm < ADVANCE_HOLONOMIC_WORLD_Y_MIN_MM) ||
        (goal->y_mm > ADVANCE_HOLONOMIC_WORLD_Y_MAX_MM) ||
        (goal->vmax_mm_s <= 0.0f) ||
        (goal->vmax_mm_s > ADVANCE_HOLONOMIC_MAX_VMAX_MM_S))) ||
      (((goal->goal_flags & ADVANCE_HOLONOMIC_GOAL_USE_YAW) != 0U) &&
       ((goal->wmax_deg_s <= 0.0f) ||
        (goal->wmax_deg_s > ADVANCE_HOLONOMIC_MAX_WMAX_DEG_S))) ||
      (goal->timeout_ms > ADVANCE_HOLONOMIC_MAX_TIMEOUT_MS) ||
      ((goal->goal_flags &
        (uint8_t)(~(ADVANCE_HOLONOMIC_GOAL_USE_YAW | ADVANCE_HOLONOMIC_GOAL_USE_POSITION))) != 0U) ||
      ((goal->goal_flags &
        (ADVANCE_HOLONOMIC_GOAL_USE_YAW | ADVANCE_HOLONOMIC_GOAL_USE_POSITION)) == 0U))
  {
    return 0U;
  }

  return 1U;
}

/* 配置校验：isfinite、加减速度大于零、增益非负且受限、scale 受限。 */
static uint8_t AdvanceHolonomic_IsConfigValid(const AdvanceHolonomic_Config_t *config)
{
  if ((config == NULL) ||
      (isfinite(config->linear_accel_mm_s2) == 0) ||
      (isfinite(config->linear_decel_mm_s2) == 0) ||
      (isfinite(config->yaw_accel_deg_s2) == 0) ||
      (isfinite(config->kp_forward) == 0) ||
      (isfinite(config->kv_forward) == 0) ||
      (isfinite(config->kp_lateral) == 0) ||
      (isfinite(config->kv_lateral) == 0) ||
      (isfinite(config->kp_yaw) == 0) ||
      (isfinite(config->kv_yaw) == 0) ||
      (isfinite(config->forward_scale) == 0) ||
      (isfinite(config->lateral_scale) == 0) ||
      (isfinite(config->yaw_scale) == 0))
  {
    return 0U;
  }

  return ((config->linear_accel_mm_s2 > 0.0f) &&
          (config->linear_accel_mm_s2 <= ADVANCE_HOLONOMIC_MAX_ACCEL_MM_S2) &&
          (config->linear_decel_mm_s2 > 0.0f) &&
          (config->linear_decel_mm_s2 <= ADVANCE_HOLONOMIC_MAX_ACCEL_MM_S2) &&
          (config->yaw_accel_deg_s2 > 0.0f) &&
          (config->yaw_accel_deg_s2 <= ADVANCE_HOLONOMIC_MAX_YAW_ACCEL_DEG_S2) &&
          (config->kp_forward >= 0.0f) && (config->kp_forward <= ADVANCE_HOLONOMIC_MAX_GAIN) &&
          (config->kv_forward >= 0.0f) && (config->kv_forward <= ADVANCE_HOLONOMIC_MAX_GAIN) &&
          (config->kp_lateral >= 0.0f) && (config->kp_lateral <= ADVANCE_HOLONOMIC_MAX_GAIN) &&
          (config->kv_lateral >= 0.0f) && (config->kv_lateral <= ADVANCE_HOLONOMIC_MAX_GAIN) &&
          (config->kp_yaw >= 0.0f) && (config->kp_yaw <= ADVANCE_HOLONOMIC_MAX_GAIN) &&
          (config->kv_yaw >= 0.0f) && (config->kv_yaw <= ADVANCE_HOLONOMIC_MAX_GAIN) &&
          (config->forward_scale >= ADVANCE_HOLONOMIC_MIN_SCALE) &&
          (config->forward_scale <= ADVANCE_HOLONOMIC_MAX_SCALE) &&
          (config->lateral_scale >= ADVANCE_HOLONOMIC_MIN_SCALE) &&
          (config->lateral_scale <= ADVANCE_HOLONOMIC_MAX_SCALE) &&
          (config->yaw_scale >= ADVANCE_HOLONOMIC_MIN_SCALE) &&
          (config->yaw_scale <= ADVANCE_HOLONOMIC_MAX_SCALE)) ? 1U : 0U;
}

/* 获取新鲜位姿：位置目标要求完整位姿，仅航向目标只要求当前航向源新鲜。 */
static AdvanceHolonomic_Status_t AdvanceHolonomic_GetFreshPose(WorldPose2D_t *pose, uint32_t now_tick)
{
  AdvanceWorld_Status_t world_status;

  if (pose == NULL)
  {
    return ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM;
  }

  if (g_holonomic.position_required != 0U)
  {
    world_status = AdvanceWorld_GetPoseCopy(pose);
    if (world_status == ADVANCE_WORLD_STATUS_NO_ORIGIN)
    {
      return ADVANCE_HOLONOMIC_STATUS_NO_ORIGIN;
    }
    if ((world_status != ADVANCE_WORLD_STATUS_OK) || (pose->valid == 0U))
    {
      return ADVANCE_HOLONOMIC_STATUS_NO_POSE;
    }
    if ((now_tick - pose->updated_tick) > ADVANCE_HOLONOMIC_POSE_TIMEOUT_MS)
    {
      return ADVANCE_HOLONOMIC_STATUS_POSE_TIMEOUT;
    }
    if ((now_tick - pose->yaw_updated_tick) > ADVANCE_HOLONOMIC_YAW_TIMEOUT_MS)
    {
      return ADVANCE_HOLONOMIC_STATUS_POSE_TIMEOUT;
    }
  }
  else
  {
    float yaw_deg;
    uint32_t updated_tick;

    (void)AdvanceWorld_GetPoseCopy(pose);
    if (AdvanceWorld_GetYawCopy(&yaw_deg, &updated_tick) != ADVANCE_WORLD_STATUS_OK)
    {
      return ADVANCE_HOLONOMIC_STATUS_NO_POSE;
    }
    if ((now_tick - updated_tick) > ADVANCE_HOLONOMIC_YAW_TIMEOUT_MS)
    {
      return ADVANCE_HOLONOMIC_STATUS_POSE_TIMEOUT;
    }
    pose->yaw_deg = yaw_deg;
    pose->yaw_updated_tick = updated_tick;
  }

  return ADVANCE_HOLONOMIC_STATUS_OK;
}

/* 速度估计：按位姿/航向时间戳增量计算并做一阶低通，首帧为零。 */
static void AdvanceHolonomic_UpdateMeasuredVelocity(const WorldPose2D_t *pose)
{
  uint32_t pose_dt_ms;
  uint32_t yaw_dt_ms;
  float pose_dt_s;
  float yaw_dt_s;
  float raw_vx_world;
  float raw_vy_world;
  float raw_wz;
  float vx_body;
  float vy_body;

  if (pose == NULL)
  {
    return;
  }

  if (g_holonomic.velocity_history_valid == 0U)
  {
    g_holonomic.last_pose = *pose;
    g_holonomic.last_pose_updated_tick = pose->updated_tick;
    g_holonomic.last_yaw_updated_tick = pose->yaw_updated_tick;
    g_holonomic.filtered_vx_world_mm_s = 0.0f;
    g_holonomic.filtered_vy_world_mm_s = 0.0f;
    g_holonomic.filtered_wz_deg_s = 0.0f;
    g_holonomic.measured.right_mm_s = 0.0f;
    g_holonomic.measured.forward_mm_s = 0.0f;
    g_holonomic.measured.wz_deg_s = 0.0f;
    g_holonomic.velocity_history_valid = 1U;
    return;
  }

  if (pose->updated_tick != g_holonomic.last_pose_updated_tick)
  {
    pose_dt_ms = pose->updated_tick - g_holonomic.last_pose_updated_tick;
    if ((pose_dt_ms > 0U) && (pose_dt_ms <= ADVANCE_HOLONOMIC_MAX_DT_MS))
    {
      pose_dt_s = (float)pose_dt_ms / 1000.0f;
      raw_vx_world = (pose->x_mm - g_holonomic.last_pose.x_mm) / pose_dt_s;
      raw_vy_world = (pose->y_mm - g_holonomic.last_pose.y_mm) / pose_dt_s;
      g_holonomic.filtered_vx_world_mm_s +=
          ADVANCE_HOLONOMIC_VEL_FILTER_ALPHA * (raw_vx_world - g_holonomic.filtered_vx_world_mm_s);
      g_holonomic.filtered_vy_world_mm_s +=
          ADVANCE_HOLONOMIC_VEL_FILTER_ALPHA * (raw_vy_world - g_holonomic.filtered_vy_world_mm_s);
    }
    g_holonomic.last_pose.x_mm = pose->x_mm;
    g_holonomic.last_pose.y_mm = pose->y_mm;
    g_holonomic.last_pose_updated_tick = pose->updated_tick;
  }

  if (pose->yaw_updated_tick != g_holonomic.last_yaw_updated_tick)
  {
    yaw_dt_ms = pose->yaw_updated_tick - g_holonomic.last_yaw_updated_tick;
    if ((yaw_dt_ms > 0U) && (yaw_dt_ms <= ADVANCE_HOLONOMIC_MAX_DT_MS))
    {
      yaw_dt_s = (float)yaw_dt_ms / 1000.0f;
      raw_wz = AdvanceWorld_WrapAngleDeg(pose->yaw_deg - g_holonomic.last_pose.yaw_deg) / yaw_dt_s;
      g_holonomic.filtered_wz_deg_s +=
          ADVANCE_HOLONOMIC_VEL_FILTER_ALPHA * (raw_wz - g_holonomic.filtered_wz_deg_s);
    }
    g_holonomic.last_pose.yaw_deg = pose->yaw_deg;
    g_holonomic.last_yaw_updated_tick = pose->yaw_updated_tick;
  }

  /* 参考/实测速度与误差必须使用同一当前实际航向进行坐标变换 */
  AdvanceWorld_WorldToBodyVelocity(g_holonomic.filtered_vx_world_mm_s,
                                   g_holonomic.filtered_vy_world_mm_s,
                                   pose->yaw_deg,
                                   &vx_body,
                                   &vy_body);
  g_holonomic.measured.right_mm_s = vx_body;
  g_holonomic.measured.forward_mm_s = vy_body;
  g_holonomic.measured.wz_deg_s = g_holonomic.filtered_wz_deg_s;
}

/*
 * 核心全向控制律：
 * v_cmd = v_ref + Kp * e_pose + Kv * e_velocity
 * 前向/横向/航向分别独立计算，无积分项；修正量先合成再限幅后与参考相加。
 */
static void AdvanceHolonomic_ComputeBodyCommand(
    const AdvanceHolonomic_Reference_t *reference,
    const WorldPose2D_t *actual_pose,
    const AdvanceHolonomic_BodyVelocity_t *actual_velocity,
    AdvanceHolonomic_BodyVelocity_t *command)
{
  float error_x_world;
  float error_y_world;
  float error_right_mm;
  float error_forward_mm;
  float ref_right_mm_s;
  float ref_forward_mm_s;
  float error_yaw_deg;
  float forward_position_correction;
  float forward_velocity_correction;
  float forward_correction;
  float lateral_position_correction;
  float lateral_velocity_correction;
  float lateral_correction;
  float yaw_position_correction;
  float yaw_velocity_correction;
  float yaw_correction;
  float command_forward_mm_s;
  float command_right_mm_s;
  float command_wz_deg_s;
  float linear_norm;
  float scale;

  if ((reference == NULL) || (actual_pose == NULL) ||
      (actual_velocity == NULL) || (command == NULL))
  {
    return;
  }

  if (g_holonomic.position_required != 0U)
  {
    /* 世界坐标误差转换到车体坐标（+X 向右，+Y 向前） */
    error_x_world = reference->x_mm - actual_pose->x_mm;
    error_y_world = reference->y_mm - actual_pose->y_mm;
    AdvanceWorld_WorldToBodyVelocity(error_x_world, error_y_world,
                                     actual_pose->yaw_deg,
                                     &error_right_mm, &error_forward_mm);
    AdvanceWorld_WorldToBodyVelocity(reference->vx_world_mm_s, reference->vy_world_mm_s,
                                     actual_pose->yaw_deg,
                                     &ref_right_mm_s, &ref_forward_mm_s);

    forward_position_correction = g_holonomic.config.kp_forward * error_forward_mm;
    forward_velocity_correction =
        g_holonomic.config.kv_forward * (ref_forward_mm_s - actual_velocity->forward_mm_s);
    forward_correction = AdvanceWorld_LimitFloat(
        forward_position_correction + forward_velocity_correction,
        -ADVANCE_HOLONOMIC_MAX_FORWARD_CORRECTION_MM_S,
        ADVANCE_HOLONOMIC_MAX_FORWARD_CORRECTION_MM_S);
    command_forward_mm_s = ref_forward_mm_s + forward_correction;

    lateral_position_correction = g_holonomic.config.kp_lateral * error_right_mm;
    lateral_velocity_correction =
        g_holonomic.config.kv_lateral * (ref_right_mm_s - actual_velocity->right_mm_s);
    lateral_correction = AdvanceWorld_LimitFloat(
        lateral_position_correction + lateral_velocity_correction,
        -ADVANCE_HOLONOMIC_MAX_LATERAL_CORRECTION_MM_S,
        ADVANCE_HOLONOMIC_MAX_LATERAL_CORRECTION_MM_S);
    command_right_mm_s = ref_right_mm_s + lateral_correction;

    /* 平移命令二维向量限幅到目标 vmax，保持运动方向不变 */
    linear_norm = sqrtf((command_right_mm_s * command_right_mm_s) +
                        (command_forward_mm_s * command_forward_mm_s));
    if ((g_holonomic.goal.vmax_mm_s > 0.0f) && (linear_norm > g_holonomic.goal.vmax_mm_s))
    {
      scale = g_holonomic.goal.vmax_mm_s / linear_norm;
      command_right_mm_s *= scale;
      command_forward_mm_s *= scale;
    }

    g_holonomic.debug.error_forward_mm = error_forward_mm;
    g_holonomic.debug.error_lateral_mm = error_right_mm;
    g_holonomic.debug.position_correction_forward_mm_s = forward_position_correction;
    g_holonomic.debug.position_correction_lateral_mm_s = lateral_position_correction;
    g_holonomic.debug.velocity_correction_forward_mm_s = forward_velocity_correction;
    g_holonomic.debug.velocity_correction_lateral_mm_s = lateral_velocity_correction;
  }
  else
  {
    command_right_mm_s = 0.0f;
    command_forward_mm_s = 0.0f;
    g_holonomic.debug.error_forward_mm = 0.0f;
    g_holonomic.debug.error_lateral_mm = 0.0f;
    g_holonomic.debug.position_correction_forward_mm_s = 0.0f;
    g_holonomic.debug.position_correction_lateral_mm_s = 0.0f;
    g_holonomic.debug.velocity_correction_forward_mm_s = 0.0f;
    g_holonomic.debug.velocity_correction_lateral_mm_s = 0.0f;
  }

  if (g_holonomic.yaw_required != 0U)
  {
    error_yaw_deg = AdvanceWorld_WrapAngleDeg(reference->yaw_deg - actual_pose->yaw_deg);
    yaw_position_correction = g_holonomic.config.kp_yaw * error_yaw_deg;
    yaw_velocity_correction =
        g_holonomic.config.kv_yaw * (reference->wz_deg_s - actual_velocity->wz_deg_s);
    yaw_correction = AdvanceWorld_LimitFloat(
        yaw_position_correction + yaw_velocity_correction,
        -ADVANCE_HOLONOMIC_MAX_YAW_CORRECTION_DEG_S,
        ADVANCE_HOLONOMIC_MAX_YAW_CORRECTION_DEG_S);
    command_wz_deg_s = reference->wz_deg_s + yaw_correction;
    if (g_holonomic.goal.wmax_deg_s > 0.0f)
    {
      command_wz_deg_s = AdvanceWorld_LimitFloat(
          command_wz_deg_s,
          -g_holonomic.goal.wmax_deg_s,
          g_holonomic.goal.wmax_deg_s);
    }

    g_holonomic.debug.error_yaw_deg = error_yaw_deg;
    g_holonomic.debug.yaw_position_correction_deg_s = yaw_position_correction;
    g_holonomic.debug.yaw_velocity_correction_deg_s = yaw_velocity_correction;
  }
  else
  {
    command_wz_deg_s = 0.0f;
    g_holonomic.debug.error_yaw_deg = 0.0f;
    g_holonomic.debug.yaw_position_correction_deg_s = 0.0f;
    g_holonomic.debug.yaw_velocity_correction_deg_s = 0.0f;
  }

  /* 驱动比例校准层：三个对角比例，底盘层负责麦轮解算与四轮同步 */
  command->right_mm_s = g_holonomic.config.lateral_scale * command_right_mm_s;
  command->forward_mm_s = g_holonomic.config.forward_scale * command_forward_mm_s;
  command->wz_deg_s = g_holonomic.config.yaw_scale * command_wz_deg_s;

  g_holonomic.debug.command_forward_mm_s = command_forward_mm_s;
  g_holonomic.debug.command_lateral_mm_s = command_right_mm_s;
  g_holonomic.debug.command_wz_deg_s = command_wz_deg_s;
  g_holonomic.debug.drive_forward_mm_s = command->forward_mm_s;
  g_holonomic.debug.drive_lateral_mm_s = command->right_mm_s;
  g_holonomic.debug.drive_wz_deg_s = command->wz_deg_s;
}

/* 到达判定：位置误差、线速度、航向误差、角速度连续保持；未启用轴不检查。 */
static uint8_t AdvanceHolonomic_CheckArrival(const WorldPose2D_t *pose, uint32_t now_tick)
{
  float error_x_world;
  float error_y_world;
  float position_error_mm;
  float linear_speed_mm_s;
  float yaw_error_deg;

  if (pose == NULL)
  {
    return 0U;
  }

  if (g_holonomic.position_required != 0U)
  {
    error_x_world = g_holonomic.goal_x_mm - pose->x_mm;
    error_y_world = g_holonomic.goal_y_mm - pose->y_mm;
    position_error_mm = sqrtf((error_x_world * error_x_world) +
                              (error_y_world * error_y_world));
    linear_speed_mm_s = sqrtf((g_holonomic.measured.right_mm_s * g_holonomic.measured.right_mm_s) +
                              (g_holonomic.measured.forward_mm_s * g_holonomic.measured.forward_mm_s));
    if ((position_error_mm > ADVANCE_HOLONOMIC_POSITION_TOLERANCE_MM) ||
        (linear_speed_mm_s > ADVANCE_HOLONOMIC_LINEAR_SPEED_TOLERANCE_MM_S))
    {
      g_holonomic.arrive_hold_start_tick = 0U;
      return 0U;
    }
  }

  if (g_holonomic.yaw_required != 0U)
  {
    yaw_error_deg = fabsf(AdvanceWorld_WrapAngleDeg(g_holonomic.goal_yaw_deg - pose->yaw_deg));
    if ((yaw_error_deg > ADVANCE_HOLONOMIC_YAW_TOLERANCE_DEG) ||
        (fabsf(g_holonomic.measured.wz_deg_s) > ADVANCE_HOLONOMIC_YAW_RATE_TOLERANCE_DEG_S))
    {
      g_holonomic.arrive_hold_start_tick = 0U;
      return 0U;
    }
  }

  if (g_holonomic.arrive_hold_start_tick == 0U)
  {
    g_holonomic.arrive_hold_start_tick = now_tick;
  }
  return ((now_tick - g_holonomic.arrive_hold_start_tick) >= ADVANCE_HOLONOMIC_ARRIVE_HOLD_MS)
             ? 1U
             : 0U;
}

/* 根据平移/航向轮廓生成连续参考位姿与参考速度。 */
static void AdvanceHolonomic_GenerateReference(AdvanceHolonomic_Reference_t *reference)
{
  if (reference == NULL)
  {
    return;
  }

  if (g_holonomic.position_required != 0U)
  {
    reference->x_mm = g_holonomic.start_x_mm +
                      (g_holonomic.direction_x * g_holonomic.linear_profile.position);
    reference->y_mm = g_holonomic.start_y_mm +
                      (g_holonomic.direction_y * g_holonomic.linear_profile.position);
    reference->vx_world_mm_s = g_holonomic.direction_x * g_holonomic.linear_profile.velocity;
    reference->vy_world_mm_s = g_holonomic.direction_y * g_holonomic.linear_profile.velocity;
  }
  else
  {
    reference->x_mm = g_holonomic.start_x_mm;
    reference->y_mm = g_holonomic.start_y_mm;
    reference->vx_world_mm_s = 0.0f;
    reference->vy_world_mm_s = 0.0f;
  }

  if (g_holonomic.yaw_required != 0U)
  {
    reference->yaw_deg = AdvanceWorld_WrapAngleDeg(
        g_holonomic.start_yaw_deg + g_holonomic.yaw_profile.position);
    reference->wz_deg_s = g_holonomic.yaw_profile.velocity;
  }
  else
  {
    reference->yaw_deg = g_holonomic.start_yaw_deg;
    reference->wz_deg_s = 0.0f;
  }
}

/* 尝试送达停车命令；失败由 stopping 状态在后续周期重试。 */
static uint8_t AdvanceHolonomic_TryStop(uint8_t hard_stop)
{
  uint8_t ok;

  if (hard_stop != 0U)
  {
    ok = (Chassis_Stop() != 0U) ? 1U : 0U;
  }
  else
  {
    ok = (Chassis_SmoothStop(g_holonomic.acc) != 0U) ? 1U : 0U;
  }
  return ok;
}

/* 进入终态：先提交明确停车命令，成功后释放控制权；失败保留控制权重试。 */
static void AdvanceHolonomic_EnterTerminalState(AdvanceHolonomic_RunState_t terminal_state,
                                                uint32_t now_tick)
{
  uint8_t hard_stop = (terminal_state == ADVANCE_HOLONOMIC_STATE_ARRIVED) ? 1U : 0U;

  (void)now_tick;
  g_holonomic.pending_terminal_state = terminal_state;
  if (AdvanceControl_GetMode() != ADVANCE_CONTROL_HOLONOMIC)
  {
    /* 控制权已不在本模块，不发送停车，直接进入终态 */
    g_holonomic.stopping = 0U;
    g_holonomic_state = terminal_state;
    return;
  }

  if (AdvanceHolonomic_TryStop(hard_stop) != 0U)
  {
    g_holonomic.stopping = 0U;
    g_holonomic_state = terminal_state;
    (void)AdvanceControl_ReleaseMode();
    return;
  }

  /* 停车入队失败：停止下发速度命令，保持控制权由后续周期重试 */
  g_holonomic.stopping = 1U;
}

/* 控制权意外丢失时清理运行时状态，不发送停车命令。 */
static void AdvanceHolonomic_ResetRunState(void)
{
  g_holonomic.linear_profile = (AdvanceHolonomic_Profile1D_t){0};
  g_holonomic.yaw_profile = (AdvanceHolonomic_Profile1D_t){0};
  g_holonomic.filtered_vx_world_mm_s = 0.0f;
  g_holonomic.filtered_vy_world_mm_s = 0.0f;
  g_holonomic.filtered_wz_deg_s = 0.0f;
  g_holonomic.last_pose = (WorldPose2D_t){0};
  g_holonomic.last_pose_updated_tick = 0U;
  g_holonomic.last_yaw_updated_tick = 0U;
  g_holonomic.velocity_history_valid = 0U;
  g_holonomic.measured.right_mm_s = 0.0f;
  g_holonomic.measured.forward_mm_s = 0.0f;
  g_holonomic.measured.wz_deg_s = 0.0f;
  g_holonomic.arrive_hold_start_tick = 0U;
  g_holonomic.stopping = 0U;
  g_holonomic.last_update_tick = HAL_GetTick();
}

/* 调试快照在一个控制周期结束时整体更新。 */
static void AdvanceHolonomic_UpdateDebugSnapshot(uint32_t now_tick)
{
  AdvanceHolonomic_DebugSnapshot_t *debug = &g_holonomic.debug;

  debug->tick = now_tick;
  debug->state = g_holonomic_state;
  debug->goal = g_holonomic.goal;
  debug->actual_pose = g_holonomic.latest_pose;
  debug->measured_forward_mm_s = g_holonomic.measured.forward_mm_s;
  debug->measured_lateral_mm_s = g_holonomic.measured.right_mm_s;
  debug->measured_wz_deg_s = g_holonomic.measured.wz_deg_s;
  debug->profile_progress_mm = g_holonomic.linear_profile.position;
  debug->profile_remaining_mm = fmaxf(
      g_holonomic.linear_profile.goal_position - g_holonomic.linear_profile.position, 0.0f);
  debug->profile_reference_speed_mm_s = g_holonomic.linear_profile.velocity;
}

/* 空闲/终态下刷新快照位姿与时间戳，供上位机与调试查询。 */
static void AdvanceHolonomic_UpdateIdleSnapshot(uint32_t now_tick)
{
  WorldPose2D_t pose = {0};

  (void)AdvanceWorld_GetPoseCopy(&pose);
  g_holonomic.latest_pose = pose;
  AdvanceHolonomic_UpdateDebugSnapshot(now_tick);
}

void AdvanceHolonomic_Init(void)
{
  g_holonomic = (AdvanceHolonomic_Module_t){
      .pending_terminal_state = ADVANCE_HOLONOMIC_STATE_IDLE};
  g_holonomic.config = g_holonomic_default;
  g_holonomic_state = ADVANCE_HOLONOMIC_STATE_IDLE;
  AdvanceHolonomic_UpdateDebugSnapshot(HAL_GetTick());
}

AdvanceHolonomic_Status_t AdvanceHolonomic_Start(const WorldGoalPose2D_t *goal,
                                                  uint8_t driver_acc)
{
  WorldPose2D_t pose = {0};
  AdvanceHolonomic_Status_t pose_status;
  uint32_t now_tick;
  float dx;
  float dy;
  float yaw_delta;

  if (AdvanceHolonomic_IsGoalValid(goal) == 0U)
  {
    return ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM;
  }
  if (AdvanceControl_GetMode() != ADVANCE_CONTROL_NONE)
  {
    return ADVANCE_HOLONOMIC_STATUS_BUSY;
  }

  g_holonomic.position_required =
      ((goal->goal_flags & ADVANCE_HOLONOMIC_GOAL_USE_POSITION) != 0U) ? 1U : 0U;
  g_holonomic.yaw_required =
      ((goal->goal_flags & ADVANCE_HOLONOMIC_GOAL_USE_YAW) != 0U) ? 1U : 0U;

  now_tick = HAL_GetTick();
  pose_status = AdvanceHolonomic_GetFreshPose(&pose, now_tick);
  if (pose_status != ADVANCE_HOLONOMIC_STATUS_OK)
  {
    return pose_status;
  }

  /* 参数与位姿全部检查通过后再占用控制权 */
  if (AdvanceControl_SetMode(ADVANCE_CONTROL_HOLONOMIC) == 0U)
  {
    return ADVANCE_HOLONOMIC_STATUS_BUSY;
  }

  g_holonomic.goal = *goal;
  g_holonomic.acc = driver_acc;
  g_holonomic.start_x_mm = pose.x_mm;
  g_holonomic.start_y_mm = pose.y_mm;
  g_holonomic.start_yaw_deg = pose.yaw_deg;

  /* 新任务启动：清空上一任务调试字段，避免残留修正量与命令 */
  g_holonomic.debug = (AdvanceHolonomic_DebugSnapshot_t){0};
  g_holonomic.debug.tick = now_tick;
  g_holonomic.debug.state = ADVANCE_HOLONOMIC_STATE_RUNNING;
  g_holonomic.debug.goal = *goal;
  g_holonomic.debug.actual_pose = pose;
  g_holonomic.debug.reference_x_mm = pose.x_mm;
  g_holonomic.debug.reference_y_mm = pose.y_mm;
  g_holonomic.debug.reference_yaw_deg = pose.yaw_deg;

  if (g_holonomic.position_required != 0U)
  {
    g_holonomic.goal_x_mm = goal->x_mm;
    g_holonomic.goal_y_mm = goal->y_mm;
    dx = goal->x_mm - pose.x_mm;
    dy = goal->y_mm - pose.y_mm;
    g_holonomic.path_length_mm = sqrtf((dx * dx) + (dy * dy));
    if (g_holonomic.path_length_mm > ADVANCE_HOLONOMIC_MIN_PATH_LENGTH_MM)
    {
      g_holonomic.direction_x = dx / g_holonomic.path_length_mm;
      g_holonomic.direction_y = dy / g_holonomic.path_length_mm;
    }
    else
    {
      g_holonomic.direction_x = 0.0f;
      g_holonomic.direction_y = 0.0f;
      g_holonomic.path_length_mm = 0.0f;
    }
    AdvanceHolonomic_Profile1D_Init(&g_holonomic.linear_profile,
                                    g_holonomic.path_length_mm,
                                    goal->vmax_mm_s,
                                    g_holonomic.config.linear_accel_mm_s2,
                                    g_holonomic.config.linear_decel_mm_s2);
  }
  else
  {
    g_holonomic.goal_x_mm = pose.x_mm;
    g_holonomic.goal_y_mm = pose.y_mm;
    g_holonomic.direction_x = 0.0f;
    g_holonomic.direction_y = 0.0f;
    g_holonomic.path_length_mm = 0.0f;
    AdvanceHolonomic_Profile1D_Init(&g_holonomic.linear_profile,
                                    0.0f,
                                    0.0f,
                                    g_holonomic.config.linear_accel_mm_s2,
                                    g_holonomic.config.linear_decel_mm_s2);
  }

  if (g_holonomic.yaw_required != 0U)
  {
    g_holonomic.goal_yaw_deg = AdvanceWorld_WrapAngleDeg(goal->yaw_deg);
    /* 内部航向目标 = 起始角 + 最短有符号增量，避免 ±180° 附近长路径旋转 */
    yaw_delta = AdvanceWorld_WrapAngleDeg(goal->yaw_deg - pose.yaw_deg);
    AdvanceHolonomic_Profile1D_Init(&g_holonomic.yaw_profile,
                                    yaw_delta,
                                    goal->wmax_deg_s,
                                    g_holonomic.config.yaw_accel_deg_s2,
                                    g_holonomic.config.yaw_accel_deg_s2);
  }
  else
  {
    g_holonomic.goal_yaw_deg = pose.yaw_deg;
    AdvanceHolonomic_Profile1D_Init(&g_holonomic.yaw_profile,
                                    0.0f,
                                    0.0f,
                                    g_holonomic.config.yaw_accel_deg_s2,
                                    g_holonomic.config.yaw_accel_deg_s2);
  }

  g_holonomic.started_tick = now_tick;
  g_holonomic.last_update_tick = now_tick;
  g_holonomic.arrive_hold_start_tick = 0U;
  g_holonomic.stopping = 0U;
  g_holonomic.velocity_history_valid = 0U;
  g_holonomic.latest_pose = pose;
  g_holonomic_state = ADVANCE_HOLONOMIC_STATE_RUNNING;

  AdvanceHolonomic_UpdateMeasuredVelocity(&pose);
  AdvanceHolonomic_UpdateDebugSnapshot(now_tick);
  return ADVANCE_HOLONOMIC_STATUS_OK;
}

void AdvanceHolonomic_Update(void)
{
  uint32_t now_tick = HAL_GetTick();
  AdvanceHolonomic_Reference_t reference = {0};
  AdvanceHolonomic_BodyVelocity_t command = {0};
  WorldPose2D_t pose = {0};
  AdvanceHolonomic_Status_t pose_status;
  uint32_t dt_ms;
  float dt_s;
  uint8_t profiles_done;

  /* 终态停车未送达时继续重试，保持控制权直至停车成功 */
  if (g_holonomic.stopping != 0U)
  {
    if (AdvanceControl_GetMode() != ADVANCE_CONTROL_HOLONOMIC)
    {
      /* 控制权意外丢失：不再发送停车命令 */
      g_holonomic.stopping = 0U;
      g_holonomic_state = ADVANCE_HOLONOMIC_STATE_CANCELED;
      AdvanceHolonomic_ResetRunState();
    }
    else if (AdvanceHolonomic_TryStop(
                 (g_holonomic.pending_terminal_state == ADVANCE_HOLONOMIC_STATE_ARRIVED)
                     ? 1U
                     : 0U) != 0U)
    {
      g_holonomic.stopping = 0U;
      g_holonomic_state = g_holonomic.pending_terminal_state;
      (void)AdvanceControl_ReleaseMode();
    }
    AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
    return;
  }

  if ((g_holonomic_state != ADVANCE_HOLONOMIC_STATE_RUNNING) &&
      (g_holonomic_state != ADVANCE_HOLONOMIC_STATE_SETTLING))
  {
    AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
    return;
  }

  /* 控制权意外丢失：停止发命令并转入 CANCELED，不发送停车 */
  if (AdvanceControl_GetMode() != ADVANCE_CONTROL_HOLONOMIC)
  {
    AdvanceHolonomic_ResetRunState();
    g_holonomic_state = ADVANCE_HOLONOMIC_STATE_CANCELED;
    AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
    return;
  }

  /* 目标超时；timeout_ms == 0 表示不启用超时 */
  if ((g_holonomic.goal.timeout_ms > 0U) &&
      ((now_tick - g_holonomic.started_tick) >= g_holonomic.goal.timeout_ms))
  {
    AdvanceHolonomic_EnterTerminalState(ADVANCE_HOLONOMIC_STATE_TIMEOUT, now_tick);
    AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
    return;
  }

  pose_status = AdvanceHolonomic_GetFreshPose(&pose, now_tick);
  if (pose_status == ADVANCE_HOLONOMIC_STATUS_NO_ORIGIN)
  {
    AdvanceHolonomic_EnterTerminalState(ADVANCE_HOLONOMIC_STATE_NO_ORIGIN, now_tick);
    AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
    return;
  }
  if (pose_status != ADVANCE_HOLONOMIC_STATUS_OK)
  {
    AdvanceHolonomic_EnterTerminalState(ADVANCE_HOLONOMIC_STATE_NO_POSE, now_tick);
    AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
    return;
  }
  g_holonomic.latest_pose = pose;

  dt_ms = now_tick - g_holonomic.last_update_tick;
  if (dt_ms == 0U)
  {
    /* 零时间间隔：不推进轮廓、不更新 last_update_tick、不下发底盘命令 */
    return;
  }
  if (dt_ms > ADVANCE_HOLONOMIC_MAX_DT_MS)
  {
    dt_ms = ADVANCE_HOLONOMIC_MAX_DT_MS;
  }
  dt_s = (float)dt_ms / 1000.0f;
  g_holonomic.last_update_tick = now_tick;

  AdvanceHolonomic_UpdateMeasuredVelocity(&pose);

  if (g_holonomic.position_required != 0U)
  {
    AdvanceHolonomic_Profile1D_Update(&g_holonomic.linear_profile, dt_s);
  }
  if (g_holonomic.yaw_required != 0U)
  {
    AdvanceHolonomic_Profile1D_Update(&g_holonomic.yaw_profile, dt_s);
  }

  profiles_done = (((g_holonomic.position_required == 0U) ||
                    (g_holonomic.linear_profile.finished != 0U)) &&
                   ((g_holonomic.yaw_required == 0U) ||
                    (g_holonomic.yaw_profile.finished != 0U)))
                      ? 1U
                      : 0U;

  if (profiles_done != 0U)
  {
    if (g_holonomic_state == ADVANCE_HOLONOMIC_STATE_RUNNING)
    {
      g_holonomic_state = ADVANCE_HOLONOMIC_STATE_SETTLING;
      g_holonomic.arrive_hold_start_tick = 0U;
    }
    /* SETTLING 参考固定为最终目标，速度归零，继续执行相同控制律 */
    reference.x_mm = g_holonomic.goal_x_mm;
    reference.y_mm = g_holonomic.goal_y_mm;
    reference.yaw_deg = g_holonomic.goal_yaw_deg;
    reference.vx_world_mm_s = 0.0f;
    reference.vy_world_mm_s = 0.0f;
    reference.wz_deg_s = 0.0f;
  }
  else
  {
    AdvanceHolonomic_GenerateReference(&reference);
  }

  g_holonomic.debug.reference_x_mm = reference.x_mm;
  g_holonomic.debug.reference_y_mm = reference.y_mm;
  g_holonomic.debug.reference_yaw_deg = reference.yaw_deg;
  g_holonomic.debug.reference_vx_world_mm_s = reference.vx_world_mm_s;
  g_holonomic.debug.reference_vy_world_mm_s = reference.vy_world_mm_s;
  g_holonomic.debug.reference_wz_deg_s = reference.wz_deg_s;

  AdvanceHolonomic_ComputeBodyCommand(&reference, &pose, &g_holonomic.measured, &command);

  /* 车体速度层输出，底盘层负责麦轮解算与 X42S 速度闭环 */
  if (Chassis_SetBodyVelocityEx(command.right_mm_s,
                                command.forward_mm_s,
                                command.wz_deg_s,
                                g_holonomic.acc) == 0U)
  {
    /* 下发失败：本周期不更新命令快照，下一周期重试 */
    AdvanceHolonomic_UpdateDebugSnapshot(now_tick);
    return;
  }

  if (g_holonomic_state == ADVANCE_HOLONOMIC_STATE_SETTLING)
  {
    if (AdvanceHolonomic_CheckArrival(&pose, now_tick) != 0U)
    {
      AdvanceHolonomic_EnterTerminalState(ADVANCE_HOLONOMIC_STATE_ARRIVED, now_tick);
      AdvanceHolonomic_UpdateIdleSnapshot(now_tick);
      return;
    }
  }

  AdvanceHolonomic_UpdateDebugSnapshot(now_tick);
}

void AdvanceHolonomic_Cancel(void)
{
  if ((g_holonomic_state != ADVANCE_HOLONOMIC_STATE_RUNNING) &&
      (g_holonomic_state != ADVANCE_HOLONOMIC_STATE_SETTLING))
  {
    return;
  }
  AdvanceHolonomic_EnterTerminalState(ADVANCE_HOLONOMIC_STATE_CANCELED, HAL_GetTick());
}

uint8_t AdvanceHolonomic_IsActive(void)
{
  return ((g_holonomic_state == ADVANCE_HOLONOMIC_STATE_RUNNING) ||
          (g_holonomic_state == ADVANCE_HOLONOMIC_STATE_SETTLING))
             ? 1U
             : 0U;
}

AdvanceHolonomic_RunState_t AdvanceHolonomic_GotoGoalBlocking(
    const WorldGoalPose2D_t *goal,
    uint8_t driver_acc)
{
  if (AdvanceHolonomic_Start(goal, driver_acc) != ADVANCE_HOLONOMIC_STATUS_OK)
  {
    return ADVANCE_HOLONOMIC_STATE_CANCELED;
  }
  while ((g_holonomic_state == ADVANCE_HOLONOMIC_STATE_RUNNING) ||
         (g_holonomic_state == ADVANCE_HOLONOMIC_STATE_SETTLING))
  {
    __WFI();
  }
  return g_holonomic_state;
}

AdvanceHolonomic_RunState_t AdvanceHolonomic_GotoPoseBlocking(float x_mm,
                                                               float y_mm,
                                                               float yaw_deg,
                                                               uint8_t driver_acc)
{
  WorldGoalPose2D_t goal = {
      .x_mm = x_mm,
      .y_mm = y_mm,
      .yaw_deg = yaw_deg,
      .vmax_mm_s = ADVANCE_HOLONOMIC_DEFAULT_VMAX_MM_S,
      .wmax_deg_s = ADVANCE_HOLONOMIC_DEFAULT_WMAX_DEG_S,
      .timeout_ms = ADVANCE_HOLONOMIC_DEFAULT_TIMEOUT_MS,
      .goal_flags = ADVANCE_HOLONOMIC_GOAL_USE_POSITION | ADVANCE_HOLONOMIC_GOAL_USE_YAW};

  return AdvanceHolonomic_GotoGoalBlocking(&goal, driver_acc);
}

AdvanceHolonomic_Status_t AdvanceHolonomic_GetStatus(AdvanceHolonomic_RuntimeStatus_t *status)
{
  uint32_t primask;
  float error_x_mm;
  float error_y_mm;

  if (status == NULL)
  {
    return ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  status->state = g_holonomic_state;
  status->goal = g_holonomic.goal;
  status->actual_pose = g_holonomic.latest_pose;
  status->started_tick = g_holonomic.started_tick;
  status->updated_tick = g_holonomic.debug.tick;
  if (g_holonomic.position_required != 0U)
  {
    error_x_mm = g_holonomic.goal_x_mm - g_holonomic.latest_pose.x_mm;
    error_y_mm = g_holonomic.goal_y_mm - g_holonomic.latest_pose.y_mm;
    status->position_error_mm = sqrtf((error_x_mm * error_x_mm) +
                                      (error_y_mm * error_y_mm));
  }
  else
  {
    status->position_error_mm = 0.0f;
  }
  if (g_holonomic.yaw_required != 0U)
  {
    status->yaw_error_deg = AdvanceWorld_WrapAngleDeg(
        g_holonomic.goal_yaw_deg - g_holonomic.latest_pose.yaw_deg);
  }
  else
  {
    status->yaw_error_deg = 0.0f;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_HOLONOMIC_STATUS_OK;
}

AdvanceHolonomic_Status_t AdvanceHolonomic_GetDebugSnapshot(
    AdvanceHolonomic_DebugSnapshot_t *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = g_holonomic.debug;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_HOLONOMIC_STATUS_OK;
}

AdvanceHolonomic_Status_t AdvanceHolonomic_GetConfig(AdvanceHolonomic_Config_t *config)
{
  uint32_t primask;

  if (config == NULL)
  {
    return ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  *config = g_holonomic.config;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_HOLONOMIC_STATUS_OK;
}

AdvanceHolonomic_Status_t AdvanceHolonomic_SetConfig(const AdvanceHolonomic_Config_t *config)
{
  uint32_t primask;

  if (AdvanceHolonomic_IsConfigValid(config) == 0U)
  {
    return ADVANCE_HOLONOMIC_STATUS_INVALID_PARAM;
  }
  if ((g_holonomic_state == ADVANCE_HOLONOMIC_STATE_RUNNING) ||
      (g_holonomic_state == ADVANCE_HOLONOMIC_STATE_SETTLING))
  {
    return ADVANCE_HOLONOMIC_STATUS_BUSY;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  g_holonomic.config = *config;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_HOLONOMIC_STATUS_OK;
}

AdvanceHolonomic_Status_t AdvanceHolonomic_RestoreDefaultConfig(void)
{
  return AdvanceHolonomic_SetConfig(&g_holonomic_default);
}
