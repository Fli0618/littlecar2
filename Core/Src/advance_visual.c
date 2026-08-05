#include "advance_visual.h"

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

static volatile AdvanceVisual_State_t g_visual_state = ADVANCE_VISUAL_STATE_IDLE;
static AdvanceVisual_Mode_t g_visual_mode = ADVANCE_VISUAL_MODE_CIRCLE;
static uint8_t g_target_type;
static uint8_t g_stable_count;
static uint32_t g_started_tick;
static uint32_t g_last_frame_tick;
static uint32_t g_last_target_tick;

#ifdef ADVANCE_VISUAL_TEST
static volatile uint8_t g_visual_transform_test_passed;
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
  return ((signed_error_x == -20.0f) && (signed_error_y == -10.0f)) ? 1U : 0U;
}
#endif

static AdvanceVisual_FrameState_t
AdvanceVisual_GetTypedTargetFrame(int16_t *center_x, int16_t *center_y)
{
  Detect_TargetList_t targets;
  uint8_t i;
  uint8_t found = 0U;
  uint8_t best_confidence = 0U;

  if (detect_get_targets(&targets) == 0U)
  {
    return ADVANCE_VISUAL_FRAME_NONE;
  }

  for (i = 0U; i < targets.count; ++i)
  {
    const Detect_Target_t *target = &targets.targets[i];

    if ((target->type != g_target_type) || (target->measured == 0U))
    {
      continue;
    }
    if ((found == 0U) || (target->confidence > best_confidence))
    {
      *center_x = target->x;
      *center_y = target->y;
      best_confidence = target->confidence;
      found = 1U;
    }
  }

  return (found != 0U) ? ADVANCE_VISUAL_FRAME_TARGET : ADVANCE_VISUAL_FRAME_NO_TARGET;
}

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

static void AdvanceVisual_SetTerminalState(AdvanceVisual_State_t state)
{
  (void)detect_stop();
  AdvanceControl_SetMode(ADVANCE_CONTROL_NONE);
  g_visual_state = state;
}

void AdvanceVisual_Init(void)
{
  g_visual_state = ADVANCE_VISUAL_STATE_IDLE;
  g_visual_mode = ADVANCE_VISUAL_MODE_CIRCLE;
  g_target_type = 0U;
  g_stable_count = 0U;
  g_started_tick = 0U;
  g_last_frame_tick = 0U;
  g_last_target_tick = 0U;

#ifdef ADVANCE_VISUAL_TEST
  g_visual_transform_test_passed = AdvanceVisual_RunTransformSelfTest();
#endif
}

void AdvanceVisual_Update(void)
{
  AdvanceVisual_FrameState_t frame_state;
  uint32_t now_tick;
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

  if (g_visual_state != ADVANCE_VISUAL_STATE_RUNNING)
  {
    return;
  }
  if (AdvanceControl_GetMode() != ADVANCE_CONTROL_VISUAL)
  {
    AdvanceVisual_SetTerminalState(ADVANCE_VISUAL_STATE_CANCELED);
    return;
  }

  now_tick = HAL_GetTick();
  if ((now_tick - g_started_tick) >= ADVANCE_VISUAL_TOTAL_TIMEOUT_MS)
  {
    AdvanceVisual_SetTerminalState(ADVANCE_VISUAL_STATE_TIMEOUT);
    return;
  }

  frame_state = AdvanceVisual_GetFrame(&center_x, &center_y);
  if (frame_state == ADVANCE_VISUAL_FRAME_NONE)
  {
    if ((now_tick - g_last_frame_tick) >= ADVANCE_VISUAL_STALE_MS)
    {
      Chassis_SmoothStop(ADVANCE_VISUAL_ACC);
    }
    if ((now_tick - g_last_target_tick) >= ADVANCE_VISUAL_LOST_TIMEOUT_MS)
    {
      AdvanceVisual_SetTerminalState(ADVANCE_VISUAL_STATE_NO_TARGET);
    }
    return;
  }

  g_last_frame_tick = now_tick;
  if (frame_state == ADVANCE_VISUAL_FRAME_NO_TARGET)
  {
    g_stable_count = 0U;
    Chassis_SmoothStop(ADVANCE_VISUAL_ACC);
    if ((now_tick - g_last_target_tick) >= ADVANCE_VISUAL_LOST_TIMEOUT_MS)
    {
      AdvanceVisual_SetTerminalState(ADVANCE_VISUAL_STATE_NO_TARGET);
    }
    return;
  }

  g_last_target_tick = now_tick;
  pixel_error_x = (int32_t)center_x - (int32_t)AdvanceVisual_GetReferenceX();
  pixel_error_y = (int32_t)center_y - (int32_t)AdvanceVisual_GetReferenceY();
  AdvanceVisual_TransformPixelError(ADVANCE_VISUAL_CAMERA_ROTATION,
                                    pixel_error_x, pixel_error_y,
                                    &body_error_x, &body_error_y);
  if ((AdvanceVisual_AbsI32(body_error_x) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_X) &&
      (AdvanceVisual_AbsI32(body_error_y) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_Y))
  {
    Chassis_SetBodyVelocityEx(0.0f, 0.0f, 0.0f, ADVANCE_VISUAL_ACC);
    ++g_stable_count;
    if (g_stable_count >= ADVANCE_VISUAL_ARRIVE_COUNT)
    {
      AdvanceVisual_SetTerminalState(ADVANCE_VISUAL_STATE_ARRIVED);
    }
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
  Chassis_SetBodyVelocityEx(vx_right, vy_forward, 0.0f, ADVANCE_VISUAL_ACC);
}

static AdvanceVisual_State_t AdvanceVisual_ReturnStartError(void)
{
  g_visual_state = ADVANCE_VISUAL_STATE_START_ERROR;
  return g_visual_state;
}

static AdvanceVisual_State_t
AdvanceVisual_RunBlockingInternal(AdvanceVisual_Mode_t mode, uint8_t target_type)
{
  Detect_Status_t status;
  uint32_t now_tick;

  /*
   * 直接尝试获取控制权，不再采用
   * GetMode() 检查后再 SetMode() 的分离流程。
   */
  if (AdvanceControl_SetMode(ADVANCE_CONTROL_VISUAL) == 0U)
  {
    return AdvanceVisual_ReturnStartError();
  }

  status = AdvanceVisual_StartDetection(mode);
  if (status != DETECT_STATUS_OK)
  {
    /*
     * Jetson 检测未成功启动，此时底盘尚未运动，
     * 直接释放视觉控制权。
     */
    AdvanceControl_ReleaseMode();
    return AdvanceVisual_ReturnStartError();
  }

  now_tick = HAL_GetTick();
  g_visual_mode = mode;
  g_target_type = target_type;
  g_stable_count = 0U;
  g_started_tick = now_tick;
  g_last_frame_tick = now_tick;
  g_last_target_tick = now_tick;
  g_visual_state = ADVANCE_VISUAL_STATE_RUNNING;

  while (g_visual_state == ADVANCE_VISUAL_STATE_RUNNING)
  {
    __WFI();
  }

  return g_visual_state;
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
    AdvanceVisual_SetTerminalState(ADVANCE_VISUAL_STATE_CANCELED);
  }
}

AdvanceVisual_State_t AdvanceVisual_GetState(void)
{
  return g_visual_state;
}
