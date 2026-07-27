#include "advance_visual.h"

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

static uint8_t AdvanceVisual_IsModeValid(AdvanceVisual_Mode_t mode)
{
  return ((mode == ADVANCE_VISUAL_MODE_CIRCLE) ||
          (mode == ADVANCE_VISUAL_MODE_COLOR) ||
          (mode == ADVANCE_VISUAL_MODE_MATERIAL))
             ? 1U
             : 0U;
}

static int16_t AdvanceVisual_GetReferenceX(void)
{
  switch (g_visual_mode)
  {
    case ADVANCE_VISUAL_MODE_CIRCLE:
      return ADVANCE_VISUAL_CIRCLE_REF_X;

    case ADVANCE_VISUAL_MODE_COLOR:
      return ADVANCE_VISUAL_COLOR_REF_X;

    default:
      return ADVANCE_VISUAL_MATERIAL_REF_X;
  }
}

static int16_t AdvanceVisual_GetReferenceY(void)
{
  switch (g_visual_mode)
  {
    case ADVANCE_VISUAL_MODE_CIRCLE:
      return ADVANCE_VISUAL_CIRCLE_REF_Y;

    case ADVANCE_VISUAL_MODE_COLOR:
      return ADVANCE_VISUAL_COLOR_REF_Y;

    default:
      return ADVANCE_VISUAL_MATERIAL_REF_Y;
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

static AdvanceVisual_FrameState_t AdvanceVisual_GetFrame(int16_t *center_x, int16_t *center_y)
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

static Detect_Status_t AdvanceVisual_StartDetection(AdvanceVisual_Mode_t mode)
{
  return (mode == ADVANCE_VISUAL_MODE_CIRCLE) ? detect_circle_start() : detect_color_start();
}

static void AdvanceVisual_SetTerminalState(AdvanceVisual_State_t state)
{
  Chassis_SmoothStop(ADVANCE_VISUAL_ACC);
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
}

void AdvanceVisual_Update(void)
{
  AdvanceVisual_FrameState_t frame_state;
  uint32_t now_tick;
  int16_t center_x = 0;
  int16_t center_y = 0;
  int32_t error_x;
  int32_t error_y;
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
  error_x = (int32_t)center_x - (int32_t)AdvanceVisual_GetReferenceX();
  error_y = (int32_t)center_y - (int32_t)AdvanceVisual_GetReferenceY();
  if ((AdvanceVisual_AbsI32(error_x) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_X) &&
      (AdvanceVisual_AbsI32(error_y) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_Y))
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
  vx_right = (AdvanceVisual_AbsI32(error_x) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_X)
                 ? 0.0f
                 : ADVANCE_VISUAL_X_SIGN * ADVANCE_VISUAL_KP_X * (float)error_x;
  vy_forward = (AdvanceVisual_AbsI32(error_y) <= (int32_t)ADVANCE_VISUAL_TOLERANCE_Y)
                   ? 0.0f
                   : ADVANCE_VISUAL_Y_SIGN * ADVANCE_VISUAL_KP_Y * (float)error_y;
  vx_right = AdvanceVisual_LimitFloat(vx_right, ADVANCE_VISUAL_MAX_VX);
  vy_forward = AdvanceVisual_LimitFloat(vy_forward, ADVANCE_VISUAL_MAX_VY);
  Chassis_SetBodyVelocityEx(vx_right, vy_forward, 0.0f, ADVANCE_VISUAL_ACC);
}

AdvanceVisual_State_t AdvanceVisual_AlignBlocking(AdvanceVisual_Mode_t mode, uint8_t target_type)
{
  Detect_Status_t status;
  uint32_t now_tick;

  if ((AdvanceVisual_IsModeValid(mode) == 0U) ||
      (AdvanceControl_GetMode() != ADVANCE_CONTROL_NONE))
  {
    g_visual_state = ADVANCE_VISUAL_STATE_START_ERROR;
    return g_visual_state;
  }

  status = AdvanceVisual_StartDetection(mode);
  if (status != DETECT_STATUS_OK)
  {
    g_visual_state = ADVANCE_VISUAL_STATE_START_ERROR;
    return g_visual_state;
  }

  now_tick = HAL_GetTick();
  g_visual_mode = mode;
  g_target_type = target_type;
  g_stable_count = 0U;
  g_started_tick = now_tick;
  g_last_frame_tick = now_tick;
  g_last_target_tick = now_tick;
  AdvanceControl_SetMode(ADVANCE_CONTROL_VISUAL);
  g_visual_state = ADVANCE_VISUAL_STATE_RUNNING;

  while (g_visual_state == ADVANCE_VISUAL_STATE_RUNNING)
  {
    __WFI();
  }

  return g_visual_state;
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
