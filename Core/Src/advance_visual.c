#include "advance_visual.h"

#include <math.h>
#include <stdio.h>

typedef enum
{
  ADVANCE_VISUAL_MODE_COLOR = 0U,
  ADVANCE_VISUAL_MODE_DISK_CENTER,
  ADVANCE_VISUAL_MODE_CIRCLE
} AdvanceVisual_Mode_t;

typedef enum
{
  ADVANCE_VISUAL_FRAME_NONE = 0U,
  ADVANCE_VISUAL_FRAME_NO_TARGET,
  ADVANCE_VISUAL_FRAME_TARGET
} AdvanceVisual_FrameState_t;

typedef struct
{
  uint32_t tick;
  AdvanceVisual_FrameState_t frame_state;
  int16_t center_x;
  int16_t center_y;
  int32_t pixel_error_x;
  int32_t pixel_error_y;
  int32_t body_error_x;
  int32_t body_error_y;
  int32_t command_vx;
  int32_t command_vy;
  uint8_t chassis_command_ok;
  uint8_t stable_count;
  uint32_t frame_age_ms;
  uint32_t target_age_ms;
} AdvanceVisual_DebugSnapshot_t;

static volatile AdvanceVisual_State_t g_visual_state = ADVANCE_VISUAL_STATE_IDLE;
static AdvanceVisual_Mode_t g_visual_mode = ADVANCE_VISUAL_MODE_CIRCLE;
static uint8_t g_target_type;
static uint8_t g_target_locked;
static int16_t g_locked_target_x;
static int16_t g_locked_target_y;
static uint8_t g_stable_count;
static uint32_t g_started_tick;
static uint32_t g_last_frame_tick;
static uint32_t g_last_target_tick;
static AdvanceVisual_DebugSnapshot_t g_visual_debug_snapshot;

#ifdef ADVANCE_VISUAL_TEST
static volatile uint8_t g_visual_transform_test_passed;
static volatile uint8_t g_visual_target_test_passed;
#endif

static int16_t AdvanceVisual_GetReferenceX(void)
{
  switch (g_visual_mode)
  {
    case ADVANCE_VISUAL_MODE_COLOR:
      return ADVANCE_VISUAL_COLOR_REF_X;

    case ADVANCE_VISUAL_MODE_DISK_CENTER:
      return ADVANCE_VISUAL_DISK_CENTER_REF_X;

    case ADVANCE_VISUAL_MODE_CIRCLE:
    default:
      return ADVANCE_VISUAL_CIRCLE_REF_X;
  }
}

static int16_t AdvanceVisual_GetReferenceY(void)
{
  switch (g_visual_mode)
  {
    case ADVANCE_VISUAL_MODE_COLOR:
      return ADVANCE_VISUAL_COLOR_REF_Y;

    case ADVANCE_VISUAL_MODE_DISK_CENTER:
      return ADVANCE_VISUAL_DISK_CENTER_REF_Y;

    case ADVANCE_VISUAL_MODE_CIRCLE:
    default:
      return ADVANCE_VISUAL_CIRCLE_REF_Y;
  }
}

static int32_t AdvanceVisual_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float AdvanceVisual_LimitFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static float AdvanceVisual_GetTargetDistance(int16_t x,
                                             int16_t y,
                                             int16_t reference_x,
                                             int16_t reference_y)
{
  int64_t delta_x = (int32_t)x - (int32_t)reference_x;
  int64_t delta_y = (int32_t)y - (int32_t)reference_y;

  return sqrtf((float)((delta_x * delta_x) + (delta_y * delta_y)));
}

/* 将相机像素误差旋转到车体坐标，随后应用车体轴方向标定。 */
static void AdvanceVisual_TransformPixelError(AdvanceVisual_CameraRotation_t rotation,
                                              int32_t pixel_error_x,
                                              int32_t pixel_error_y,
                                              int32_t *body_error_x,
                                              int32_t *body_error_y)
{
  switch (rotation)
  {
    case ADVANCE_VISUAL_CAMERA_ROTATION_90_CW:
      *body_error_x = pixel_error_y;
      *body_error_y = -pixel_error_x;
      break;

    case ADVANCE_VISUAL_CAMERA_ROTATION_180:
      *body_error_x = -pixel_error_x;
      *body_error_y = -pixel_error_y;
      break;

    case ADVANCE_VISUAL_CAMERA_ROTATION_270_CW:
      *body_error_x = -pixel_error_y;
      *body_error_y = pixel_error_x;
      break;

    case ADVANCE_VISUAL_CAMERA_ROTATION_0:
    default:
      *body_error_x = pixel_error_x;
      *body_error_y = pixel_error_y;
      break;
  }
}

static void AdvanceVisual_ApplyBodyAxisSign(int32_t body_error_x,
                                            int32_t body_error_y,
                                            float body_x_sign,
                                            float body_y_sign,
                                            float *signed_error_x,
                                            float *signed_error_y)
{
  *signed_error_x = body_x_sign * (float)body_error_x;
  *signed_error_y = body_y_sign * (float)body_error_y;
}

#ifdef ADVANCE_VISUAL_TEST
static uint8_t AdvanceVisual_RunTransformSelfTest(void)
{
  int32_t body_error_x;
  int32_t body_error_y;
  float signed_error_x;
  float signed_error_y;

  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION_0,
                                    10, 20, &body_error_x, &body_error_y);
  if ((body_error_x != 10) || (body_error_y != 20))
  {
    return 0U;
  }

  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION_90_CW,
                                    10, 20, &body_error_x, &body_error_y);
  if ((body_error_x != 20) || (body_error_y != -10))
  {
    return 0U;
  }

  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION_180,
                                    10, 20, &body_error_x, &body_error_y);
  if ((body_error_x != -10) || (body_error_y != -20))
  {
    return 0U;
  }

  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION_270_CW,
                                    10, 20, &body_error_x, &body_error_y);
  if ((body_error_x != -20) || (body_error_y != 10))
  {
    return 0U;
  }

  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION_90_CW,
                                    10, 20, &body_error_x, &body_error_y);
  AdvanceVisual_ApplyBodyAxisSign(body_error_x, body_error_y,
                                  -1.0f, 1.0f,
                                  &signed_error_x, &signed_error_y);
  if ((signed_error_x != -20.0f) || (signed_error_y != -10.0f))
  {
    return 0U;
  }

  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION,
                                    10, 20, &body_error_x, &body_error_y);
  AdvanceVisual_ApplyBodyAxisSign(body_error_x, body_error_y,
                                  ADVANCE_VISUAL_BODY_X_SIGN,
                                  ADVANCE_VISUAL_BODY_Y_SIGN,
                                  &signed_error_x, &signed_error_y);
  return ((signed_error_x == -20.0f) && (signed_error_y == -10.0f)) ? 1U : 0U;
}
#endif

static AdvanceVisual_FrameState_t
AdvanceVisual_SelectTypedTarget(const Detect_TargetList_t *targets,
                                uint8_t target_type,
                                int16_t reference_x,
                                int16_t reference_y,
                                uint8_t lock_valid,
                                int16_t lock_x,
                                int16_t lock_y,
                                int16_t *center_x,
                                int16_t *center_y)
{
  const Detect_Target_t *best_target = NULL;
  float best_distance = 0.0f;
  uint8_t target_count;
  uint8_t i;

  if ((targets == NULL) || (center_x == NULL) || (center_y == NULL))
  {
    return ADVANCE_VISUAL_FRAME_NO_TARGET;
  }

  target_count = (targets->count > DETECT_TARGET_MAX)
                     ? DETECT_TARGET_MAX
                     : targets->count;
  for (i = 0U; i < target_count; ++i)
  {
    const Detect_Target_t *target = &targets->targets[i];
    int16_t origin_x = (lock_valid != 0U) ? lock_x : reference_x;
    int16_t origin_y = (lock_valid != 0U) ? lock_y : reference_y;
    float distance;
    float distance_delta;

    if ((target->type != target_type) || (target->measured == 0U))
    {
      continue;
    }

    distance = AdvanceVisual_GetTargetDistance(target->x, target->y,
                                               origin_x, origin_y);
    if ((lock_valid != 0U) &&
        (distance > (float)ADVANCE_VISUAL_TARGET_LOCK_MAX_JUMP_PX))
    {
      continue;
    }

    if (best_target == NULL)
    {
      best_target = target;
      best_distance = distance;
      continue;
    }

    distance_delta = distance - best_distance;
    if (distance_delta < -(float)ADVANCE_VISUAL_TARGET_DISTANCE_TIE_PX)
    {
      best_target = target;
      best_distance = distance;
    }
    else if ((fabsf(distance_delta) <=
              (float)ADVANCE_VISUAL_TARGET_DISTANCE_TIE_PX) &&
             (target->confidence > best_target->confidence))
    {
      best_target = target;
      best_distance = distance;
    }
  }

  if (best_target == NULL)
  {
    return ADVANCE_VISUAL_FRAME_NO_TARGET;
  }

  *center_x = best_target->x;
  *center_y = best_target->y;
  return ADVANCE_VISUAL_FRAME_TARGET;
}

static AdvanceVisual_FrameState_t
AdvanceVisual_GetTypedTargetFrame(int16_t *center_x, int16_t *center_y)
{
  Detect_TargetList_t targets;
  AdvanceVisual_FrameState_t frame_state;

  if (detect_get_targets(&targets) == 0U)
  {
    return ADVANCE_VISUAL_FRAME_NONE;
  }

  frame_state = AdvanceVisual_SelectTypedTarget(
      &targets, g_target_type,
      AdvanceVisual_GetReferenceX(), AdvanceVisual_GetReferenceY(),
      g_target_locked, g_locked_target_x, g_locked_target_y,
      center_x, center_y);

  if (frame_state == ADVANCE_VISUAL_FRAME_TARGET)
  {
    g_target_locked = 1U;
    g_locked_target_x = *center_x;
    g_locked_target_y = *center_y;
  }

  return frame_state;
}

#ifdef ADVANCE_VISUAL_TEST
static uint8_t AdvanceVisual_RunTargetSelectionSelfTest(void)
{
  Detect_TargetList_t targets = {0};
  int16_t center_x = 0;
  int16_t center_y = 0;

  targets.count = 4U;
  targets.targets[0] = (Detect_Target_t){1U, 330, 240, 10U, 1U, 0U};
  targets.targets[1] = (Detect_Target_t){1U, 340, 240, 90U, 1U, 0U};
  targets.targets[2] = (Detect_Target_t){2U, 320, 240, 100U, 1U, 0U};
  targets.targets[3] = (Detect_Target_t){1U, 325, 240, 100U, 0U, 0U};
  if ((AdvanceVisual_SelectTypedTarget(&targets, 1U, 320, 240,
                                       0U, 0, 0,
                                       &center_x, &center_y) !=
       ADVANCE_VISUAL_FRAME_TARGET) ||
      (center_x != 330) || (center_y != 240))
  {
    return 0U;
  }

  targets.count = 2U;
  targets.targets[0] = (Detect_Target_t){1U, 335, 240, 10U, 1U, 0U};
  targets.targets[1] = (Detect_Target_t){1U, 350, 240, 90U, 1U, 0U};
  if ((AdvanceVisual_SelectTypedTarget(&targets, 1U, 320, 240,
                                       1U, 330, 240,
                                       &center_x, &center_y) !=
       ADVANCE_VISUAL_FRAME_TARGET) ||
      (center_x != 335) || (center_y != 240))
  {
    return 0U;
  }

  targets.count = 2U;
  targets.targets[0] = (Detect_Target_t){1U, 338, 240, 10U, 1U, 0U};
  targets.targets[1] = (Detect_Target_t){1U, 330, 248, 90U, 1U, 0U};
  if ((AdvanceVisual_SelectTypedTarget(&targets, 1U, 320, 240,
                                       1U, 330, 240,
                                       &center_x, &center_y) !=
       ADVANCE_VISUAL_FRAME_TARGET) ||
      (center_x != 330) || (center_y != 248))
  {
    return 0U;
  }

  targets.count = 1U;
  targets.targets[0] = (Detect_Target_t){1U, 451, 240, 100U, 1U, 0U};
  return (AdvanceVisual_SelectTypedTarget(&targets, 1U, 320, 240,
                                          1U, 330, 240,
                                          &center_x, &center_y) ==
          ADVANCE_VISUAL_FRAME_NO_TARGET) ? 1U : 0U;
}
#endif

static AdvanceVisual_FrameState_t
AdvanceVisual_GetDiskCenterFrame(int16_t *center_x, int16_t *center_y)
{
  Detect_DiskCenter_t result;

  if (detect_get_disk_center(&result) == 0U)
  {
    return ADVANCE_VISUAL_FRAME_NONE;
  }
  if (result.status != DETECT_STATUS_OK)
  {
    return ADVANCE_VISUAL_FRAME_NO_TARGET;
  }

  *center_x = result.x;
  *center_y = result.y;
  return ADVANCE_VISUAL_FRAME_TARGET;
}

static AdvanceVisual_FrameState_t
AdvanceVisual_GetFrame(int16_t *center_x, int16_t *center_y)
{
  switch (g_visual_mode)
  {
    case ADVANCE_VISUAL_MODE_COLOR:
    case ADVANCE_VISUAL_MODE_CIRCLE:
      return AdvanceVisual_GetTypedTargetFrame(center_x, center_y);

    case ADVANCE_VISUAL_MODE_DISK_CENTER:
      return AdvanceVisual_GetDiskCenterFrame(center_x, center_y);

    default:
      return ADVANCE_VISUAL_FRAME_NONE;
  }
}

static Detect_Status_t AdvanceVisual_StartDetection(AdvanceVisual_Mode_t mode)
{
  switch (mode)
  {
    case ADVANCE_VISUAL_MODE_COLOR:
      return detect_color_start();

    case ADVANCE_VISUAL_MODE_DISK_CENTER:
      return detect_disk_center_start();

    case ADVANCE_VISUAL_MODE_CIRCLE:
      return detect_circle_start();

    default:
      return DETECT_STATUS_BAD_PARAMETER;
  }
}

static uint8_t AdvanceVisual_StopAndReleaseControl(void)
{
  AdvanceControl_Mode_t control_mode = AdvanceControl_GetMode();
  uint8_t stop_ok;

  if (control_mode == ADVANCE_CONTROL_VISUAL)
  {
    /* 先检查硬停止命令，再仅释放控制权，避免丢失停止结果。 */
    stop_ok = Chassis_Stop();
    if (stop_ok == 0U)
    {
      /* 队列短暂繁忙时再尝试一次，阻塞接口不会带着旧速度退出。 */
      stop_ok = Chassis_Stop();
    }
    (void)AdvanceControl_ReleaseMode();
    return stop_ok;
  }

  if (control_mode == ADVANCE_CONTROL_NONE)
  {
    return Chassis_Stop();
  }

  /* 其他控制器已经接管底盘，不能覆盖其新命令。 */
  return 1U;
}

static void AdvanceVisual_Finish(AdvanceVisual_State_t state)
{
  Detect_Status_t detect_status;
  uint8_t stop_ok;

  if (g_visual_state != ADVANCE_VISUAL_STATE_RUNNING)
  {
    return;
  }

  detect_status = detect_stop();
  stop_ok = AdvanceVisual_StopAndReleaseControl();
  g_target_locked = 0U;
  g_visual_state = state;

#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  if ((detect_status != DETECT_STATUS_OK) || (stop_ok == 0U))
  {
    printf("[VIS] finish detect_status=%u stop_ok=%u\r\n",
           (unsigned int)detect_status,
           (unsigned int)stop_ok);
  }
#else
  (void)detect_status;
  (void)stop_ok;
#endif
}

void AdvanceVisual_Init(void)
{
  g_visual_state = ADVANCE_VISUAL_STATE_IDLE;
  g_visual_mode = ADVANCE_VISUAL_MODE_CIRCLE;
  g_target_type = 0U;
  g_target_locked = 0U;
  g_locked_target_x = 0;
  g_locked_target_y = 0;
  g_stable_count = 0U;
  g_started_tick = 0U;
  g_last_frame_tick = 0U;
  g_last_target_tick = 0U;
  g_visual_debug_snapshot = (AdvanceVisual_DebugSnapshot_t){0};

#ifdef ADVANCE_VISUAL_TEST
  g_visual_transform_test_passed = AdvanceVisual_RunTransformSelfTest();
  g_visual_target_test_passed = AdvanceVisual_RunTargetSelectionSelfTest();
#endif
}

static void AdvanceVisual_ControlStep(uint32_t now_tick)
{
  AdvanceVisual_FrameState_t frame_state;
  int16_t center_x = 0;
  int16_t center_y = 0;
  int32_t pixel_error_x;
  int32_t pixel_error_y;
  int32_t body_error_x;
  int32_t body_error_y;
  float signed_error_x;
  float signed_error_y;
  float vx_right;
  float vy_forward;
  uint8_t chassis_command_ok = 1U;

  if (g_visual_state != ADVANCE_VISUAL_STATE_RUNNING)
  {
    return;
  }
  if (AdvanceControl_GetMode() != ADVANCE_CONTROL_VISUAL)
  {
    AdvanceVisual_Finish(ADVANCE_VISUAL_STATE_CANCELED);
    return;
  }

  if ((now_tick - g_started_tick) >= ADVANCE_VISUAL_TOTAL_TIMEOUT_MS)
  {
    AdvanceVisual_Finish(ADVANCE_VISUAL_STATE_TIMEOUT);
    return;
  }

  frame_state = AdvanceVisual_GetFrame(&center_x, &center_y);
  g_visual_debug_snapshot.tick = now_tick;
  g_visual_debug_snapshot.frame_state = frame_state;
  g_visual_debug_snapshot.center_x = center_x;
  g_visual_debug_snapshot.center_y = center_y;
  g_visual_debug_snapshot.pixel_error_x = 0;
  g_visual_debug_snapshot.pixel_error_y = 0;
  g_visual_debug_snapshot.body_error_x = 0;
  g_visual_debug_snapshot.body_error_y = 0;
  g_visual_debug_snapshot.command_vx = 0;
  g_visual_debug_snapshot.command_vy = 0;
  g_visual_debug_snapshot.chassis_command_ok = 1U;
  g_visual_debug_snapshot.stable_count = g_stable_count;
  g_visual_debug_snapshot.frame_age_ms = now_tick - g_last_frame_tick;
  g_visual_debug_snapshot.target_age_ms = now_tick - g_last_target_tick;

  if (frame_state == ADVANCE_VISUAL_FRAME_NONE)
  {
    if ((now_tick - g_last_frame_tick) >= ADVANCE_VISUAL_STALE_MS)
    {
      chassis_command_ok = Chassis_SmoothStop(ADVANCE_VISUAL_ACC);
    }
    if ((now_tick - g_last_target_tick) >= ADVANCE_VISUAL_LOST_TIMEOUT_MS)
    {
      AdvanceVisual_Finish(ADVANCE_VISUAL_STATE_NO_TARGET);
    }
    g_visual_debug_snapshot.chassis_command_ok = chassis_command_ok;
    return;
  }

  g_last_frame_tick = now_tick;
  g_visual_debug_snapshot.frame_age_ms = 0U;
  if (frame_state == ADVANCE_VISUAL_FRAME_NO_TARGET)
  {
    g_stable_count = 0U;
    chassis_command_ok = Chassis_SmoothStop(ADVANCE_VISUAL_ACC);
    if ((now_tick - g_last_target_tick) >= ADVANCE_VISUAL_LOST_TIMEOUT_MS)
    {
      AdvanceVisual_Finish(ADVANCE_VISUAL_STATE_NO_TARGET);
    }
    g_visual_debug_snapshot.chassis_command_ok = chassis_command_ok;
    g_visual_debug_snapshot.stable_count = g_stable_count;
    g_visual_debug_snapshot.target_age_ms = now_tick - g_last_target_tick;
    return;
  }

  g_last_target_tick = now_tick;
  g_visual_debug_snapshot.target_age_ms = 0U;
  pixel_error_x = (int32_t)center_x - (int32_t)AdvanceVisual_GetReferenceX();
  pixel_error_y = (int32_t)center_y - (int32_t)AdvanceVisual_GetReferenceY();
  g_visual_debug_snapshot.pixel_error_x = pixel_error_x;
  g_visual_debug_snapshot.pixel_error_y = pixel_error_y;
  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION,
                                    pixel_error_x, pixel_error_y,
                                    &body_error_x, &body_error_y);
  g_visual_debug_snapshot.body_error_x = body_error_x;
  g_visual_debug_snapshot.body_error_y = body_error_y;
  if ((AdvanceVisual_AbsI32(body_error_x) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_X) &&
      (AdvanceVisual_AbsI32(body_error_y) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_Y))
  {
    chassis_command_ok = Chassis_SetBodyVelocityEx(0.0f, 0.0f, 0.0f,
                                                    ADVANCE_VISUAL_ACC);
    ++g_stable_count;
    if (g_stable_count >= ADVANCE_VISUAL_ARRIVE_COUNT)
    {
      AdvanceVisual_Finish(ADVANCE_VISUAL_STATE_ARRIVED);
    }
    g_visual_debug_snapshot.chassis_command_ok = chassis_command_ok;
    g_visual_debug_snapshot.stable_count = g_stable_count;
    return;
  }

  g_stable_count = 0U;
  AdvanceVisual_ApplyBodyAxisSign(body_error_x, body_error_y,
                                  ADVANCE_VISUAL_BODY_X_SIGN,
                                  ADVANCE_VISUAL_BODY_Y_SIGN,
                                  &signed_error_x, &signed_error_y);
  vx_right = (AdvanceVisual_AbsI32(body_error_x) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_X)
                 ? 0.0f
                 : ADVANCE_VISUAL_KP_X * signed_error_x;
  vy_forward = (AdvanceVisual_AbsI32(body_error_y) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_Y)
                   ? 0.0f
                   : ADVANCE_VISUAL_KP_Y * signed_error_y;
  vx_right = AdvanceVisual_LimitFloat(vx_right, ADVANCE_VISUAL_MAX_VX);
  vy_forward = AdvanceVisual_LimitFloat(vy_forward, ADVANCE_VISUAL_MAX_VY);
  chassis_command_ok = Chassis_SetBodyVelocityEx(vx_right, vy_forward, 0.0f,
                                                  ADVANCE_VISUAL_ACC);
  g_visual_debug_snapshot.command_vx = (int32_t)vx_right;
  g_visual_debug_snapshot.command_vy = (int32_t)vy_forward;
  g_visual_debug_snapshot.chassis_command_ok = chassis_command_ok;
  g_visual_debug_snapshot.stable_count = g_stable_count;
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE == 0U)
  (void)g_visual_debug_snapshot;
#endif
}

static void AdvanceVisual_LogSnapshot(void)
{
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  const AdvanceVisual_DebugSnapshot_t *snapshot = &g_visual_debug_snapshot;

  switch (snapshot->frame_state)
  {
    case ADVANCE_VISUAL_FRAME_TARGET:
      printf("[VIS] t=%lu frame=TARGET center=(%d,%d) pixel=(%ld,%ld) "
             "body=(%ld,%ld) cmd=(%ld,%ld) cmd_ok=%u stable=%u "
             "frame_age=%lu target_age=%lu\r\n",
             (unsigned long)snapshot->tick,
             (int)snapshot->center_x,
             (int)snapshot->center_y,
             (long)snapshot->pixel_error_x,
             (long)snapshot->pixel_error_y,
             (long)snapshot->body_error_x,
             (long)snapshot->body_error_y,
             (long)snapshot->command_vx,
             (long)snapshot->command_vy,
             (unsigned int)snapshot->chassis_command_ok,
             (unsigned int)snapshot->stable_count,
             (unsigned long)snapshot->frame_age_ms,
             (unsigned long)snapshot->target_age_ms);
      break;

    case ADVANCE_VISUAL_FRAME_NO_TARGET:
      printf("[VIS] t=%lu frame=NO_TARGET target=%u target_age=%lu\r\n",
             (unsigned long)snapshot->tick,
             (unsigned int)g_target_type,
             (unsigned long)snapshot->target_age_ms);
      break;

    case ADVANCE_VISUAL_FRAME_NONE:
    default:
      printf("[VIS] t=%lu frame=NONE frame_age=%lu target_age=%lu\r\n",
             (unsigned long)snapshot->tick,
             (unsigned long)snapshot->frame_age_ms,
             (unsigned long)snapshot->target_age_ms);
      break;
  }
#endif
}

static AdvanceVisual_State_t AdvanceVisual_ReturnStartError(void)
{
  g_visual_state = ADVANCE_VISUAL_STATE_START_ERROR;
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  printf("[VIS] exit state=%u elapsed=0 frame_age=0 target_age=0 "
         "stable=%u control=%u detect_active=%u\r\n",
         (unsigned int)g_visual_state,
         (unsigned int)g_stable_count,
         (unsigned int)AdvanceControl_GetMode(),
         (unsigned int)detect_is_active());
#endif
  return g_visual_state;
}

static AdvanceVisual_State_t
AdvanceVisual_RunBlockingInternal(AdvanceVisual_Mode_t mode, uint8_t target_type)
{
  Detect_Status_t status;
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  AdvanceControl_Mode_t control_before;
#endif
  AdvanceVisual_State_t final_state;
  uint32_t now_tick;
  uint32_t next_control_tick;
  uint32_t next_log_tick;
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  uint32_t elapsed_tick;
#endif

#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  control_before = AdvanceControl_GetMode();
  printf("[VIS] start mode=%u target=%u control_before=%u\r\n",
         (unsigned int)mode,
         (unsigned int)target_type,
         (unsigned int)control_before);
#endif

  /*
   * 直接尝试获取控制权，不再采用
   * GetMode() 检查后再 SetMode() 的分离流程。
   */
  if (AdvanceControl_SetMode(ADVANCE_CONTROL_VISUAL) == 0U)
  {
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
    printf("[VIS] control acquire failed\r\n");
#endif
    return AdvanceVisual_ReturnStartError();
  }

#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  printf("[VIS] control acquired\r\n");
#endif

  status = AdvanceVisual_StartDetection(mode);
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  printf("[VIS] detect_start status=%u\r\n", (unsigned int)status);
#endif
  if (status != DETECT_STATUS_OK)
  {
    /*
     * 即使启动失败也通过统一硬停止路径清除可能残留的底盘命令，
     * 不让失败路径留下 VISUAL 控制权。
     */
    (void)AdvanceVisual_StopAndReleaseControl();
    return AdvanceVisual_ReturnStartError();
  }

  now_tick = HAL_GetTick();
  g_visual_mode = mode;
  g_target_type = target_type;
  g_target_locked = 0U;
  g_locked_target_x = 0;
  g_locked_target_y = 0;
  g_stable_count = 0U;
  g_started_tick = now_tick;
  g_last_frame_tick = now_tick;
  g_last_target_tick = now_tick;
  g_visual_debug_snapshot = (AdvanceVisual_DebugSnapshot_t){0};
  g_visual_state = ADVANCE_VISUAL_STATE_RUNNING;

#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  printf("[VIS] running ref=(%d,%d) period=%lu\r\n",
         (int)AdvanceVisual_GetReferenceX(),
         (int)AdvanceVisual_GetReferenceY(),
         (unsigned long)ADVANCE_VISUAL_CONTROL_PERIOD_MS);
#endif

  next_control_tick = now_tick;
  next_log_tick = now_tick + ADVANCE_VISUAL_LOG_PERIOD_MS;

  while (g_visual_state == ADVANCE_VISUAL_STATE_RUNNING)
  {
    now_tick = HAL_GetTick();

    if ((int32_t)(now_tick - next_control_tick) >= 0)
    {
      AdvanceVisual_ControlStep(now_tick);
      /* 以本次实际执行时间重新计算，禁止延迟后的周期补跑。 */
      next_control_tick = now_tick + ADVANCE_VISUAL_CONTROL_PERIOD_MS;
    }

    if ((g_visual_state == ADVANCE_VISUAL_STATE_RUNNING) &&
        ((int32_t)(now_tick - next_log_tick) >= 0))
    {
      AdvanceVisual_LogSnapshot();
      next_log_tick = now_tick + ADVANCE_VISUAL_LOG_PERIOD_MS;
    }

    if (g_visual_state == ADVANCE_VISUAL_STATE_RUNNING)
    {
      __WFI();
    }
  }

  final_state = g_visual_state;
#if (ADVANCE_VISUAL_DEBUG_LOG_ENABLE != 0U)
  elapsed_tick = HAL_GetTick() - g_started_tick;
  printf("[VIS] exit state=%u elapsed=%lu frame_age=%lu target_age=%lu "
         "stable=%u control=%u detect_active=%u\r\n",
         (unsigned int)final_state,
         (unsigned long)elapsed_tick,
         (unsigned long)g_visual_debug_snapshot.frame_age_ms,
         (unsigned long)g_visual_debug_snapshot.target_age_ms,
         (unsigned int)g_stable_count,
         (unsigned int)AdvanceControl_GetMode(),
         (unsigned int)detect_is_active());
#endif
  return final_state;
}

AdvanceVisual_State_t AdvanceVisual_AlignColorBlocking(ColorType_t color)
{
  if ((uint32_t)color > (uint32_t)EMPTY_SLOT)
  {
    return AdvanceVisual_ReturnStartError();
  }

  return AdvanceVisual_RunBlockingInternal(
      ADVANCE_VISUAL_MODE_COLOR,
      (uint8_t)color);
}

AdvanceVisual_State_t AdvanceVisual_AlignDiskCenterBlocking(void)
{
  return AdvanceVisual_RunBlockingInternal(
      ADVANCE_VISUAL_MODE_DISK_CENTER,
      0U);
}

AdvanceVisual_State_t AdvanceVisual_AlignCircleBlocking(CircleType_t number)
{
  if ((uint32_t)number > (uint32_t)NUMBER_3)
  {
    return AdvanceVisual_ReturnStartError();
  }

  return AdvanceVisual_RunBlockingInternal(
      ADVANCE_VISUAL_MODE_CIRCLE,
      (uint8_t)number);
}

void AdvanceVisual_Cancel(void)
{
  if (g_visual_state == ADVANCE_VISUAL_STATE_RUNNING)
  {
    /* 当前工程仅从主线程调用 Cancel，复杂清理不在中断上下文执行。 */
    AdvanceVisual_Finish(ADVANCE_VISUAL_STATE_CANCELED);
  }
}

AdvanceVisual_State_t AdvanceVisual_GetState(void)
{
  return g_visual_state;
}
