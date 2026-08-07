#include "advance_material_task.h"

#include <stddef.h>

#include "advance_arm.h"
#include "advance_visual.h"

#define ADVANCE_MATERIAL_POSITION_LEFT ((uint8_t)1U)
#define ADVANCE_MATERIAL_POSITION_CENTER ((uint8_t)2U)
#define ADVANCE_MATERIAL_POSITION_RIGHT ((uint8_t)3U)

static bool AdvanceMaterialTask_ParseDigit(char value,
                                            uint8_t minimum,
                                            uint8_t maximum,
                                            uint8_t *result)
{
  uint8_t parsed;

  if ((result == NULL) || (value < '0') || (value > '9'))
  {
    return false;
  }

  parsed = (uint8_t)(value - '0');
  if ((parsed < minimum) || (parsed > maximum))
  {
    return false;
  }

  *result = parsed;
  return true;
}

static void AdvanceMaterialTask_SelectTraySlot(uint8_t slot)
{
  switch (slot)
  {
  case 1U:
    AdvanceArm_TraySlot1();
    break;

  case 2U:
    AdvanceArm_TraySlot2();
    break;

  case 3U:
    AdvanceArm_TraySlot3();
    break;

  default:
    break;
  }
}

static void AdvanceMaterialTask_RotateOutward(uint8_t position)
{
  switch (position)
  {
  case ADVANCE_MATERIAL_POSITION_LEFT:
    AdvanceArm_RotateOutwardLeft();
    break;

  case ADVANCE_MATERIAL_POSITION_CENTER:
    AdvanceArm_RotateOutwardCenter();
    break;

  case ADVANCE_MATERIAL_POSITION_RIGHT:
    AdvanceArm_RotateOutwardRight();
    break;

  default:
    break;
  }
}

static ColorType_t AdvanceMaterialTask_ToVisualColor(uint8_t color)
{
  return (ColorType_t)(color - 1U);
}

static void AdvanceMaterialTask_WaitColorAtPickupBlocking(ColorType_t color)
{
  Detect_TargetList_t targets = {0};
  uint8_t index;

  while (1)
  {
    if ((detect_get_targets(&targets) != 0U) &&
        (detect_is_fresh(ADVANCE_VISUAL_STALE_MS) != 0U))
    {
      for (index = 0U; index < targets.count; ++index)
      {
        const Detect_Target_t *target = &targets.targets[index];
        int32_t error_x;
        int32_t error_y;

        if ((target->type != (uint8_t)color) ||
            (target->measured == 0U) ||
            (target->support_count < ADVANCE_VISUAL_ARRIVE_COUNT))
        {
          continue;
        }

        error_x = (int32_t)target->x - (int32_t)ADVANCE_VISUAL_COLOR_REF_X;
        error_y = (int32_t)target->y - (int32_t)ADVANCE_VISUAL_COLOR_REF_Y;
        if (error_x < 0)
        {
          error_x = -error_x;
        }
        if (error_y < 0)
        {
          error_y = -error_y;
        }

        if ((error_x <= (int32_t)ADVANCE_VISUAL_TOLERANCE_X) &&
            (error_y <= (int32_t)ADVANCE_VISUAL_TOLERANCE_Y))
        {
          return;
        }
      }
    }

    __WFI();
  }
}

static void AdvanceMaterialTask_PlaceInTray(uint8_t tray_slot)
{
  AdvanceMaterialTask_SelectTraySlot(tray_slot);
  AdvanceArm_RotateToTray();
  AdvanceArm_LiftToTrayBlocking();
  AdvanceArm_GripperOpen();
  AdvanceArm_LiftHighBlocking();
}

static void AdvanceMaterialTask_PrepareTurntablePickup(void)
{
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  AdvanceArm_GripperOpen();
  AdvanceArm_RotateOutwardCenter();
  AdvanceArm_SlideToPickupBlocking();
}

static void AdvanceMaterialTask_PickTurntableToTray(uint8_t tray_slot)
{
  AdvanceArm_LiftToPickupBlocking();
  AdvanceArm_GripperClose();
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  AdvanceMaterialTask_PlaceInTray(tray_slot);
}

static void AdvanceMaterialTask_PickPositionToTray(uint8_t position,
                                                   uint8_t tray_slot)
{
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  AdvanceArm_GripperOpen();
  AdvanceMaterialTask_RotateOutward(position);
  AdvanceArm_SlideToPickupBlocking();
  AdvanceArm_LiftLowBlocking();
  AdvanceArm_GripperClose();
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  AdvanceMaterialTask_PlaceInTray(tray_slot);
}

static void AdvanceMaterialTask_PlaceTrayToPosition(uint8_t tray_slot,
                                                    uint8_t position,
                                                    bool stacking)
{
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  AdvanceMaterialTask_SelectTraySlot(tray_slot);
  AdvanceArm_RotateToTray();
  AdvanceArm_LiftToTrayBlocking();
  AdvanceArm_GripperClose();
  AdvanceArm_LiftHighBlocking();
  AdvanceMaterialTask_RotateOutward(position);
  AdvanceArm_SlideToPickupBlocking();

  if (stacking)
  {
    AdvanceArm_LiftToStackBlocking();
  }
  else
  {
    AdvanceArm_LiftLowBlocking();
  }

  AdvanceArm_GripperOpen();
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
}

static void AdvanceMaterialTask_Collect(
    const uint8_t colors[ADVANCE_MATERIAL_TASK_ITEM_COUNT])
{
  uint8_t index;

  /* 底盘只对准一次原料转盘中心，后续取料过程中保持不动。 */
  (void)AdvanceVisual_AlignDiskCenterBlocking();
  HAL_Delay(10U);

  /* 机械臂进入固定取料姿态，之后等待目标颜色转到该位置。 */
  AdvanceMaterialTask_PrepareTurntablePickup();
  (void)detect_color_start();

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    AdvanceMaterialTask_WaitColorAtPickupBlocking(
        AdvanceMaterialTask_ToVisualColor(colors[index]));
    AdvanceMaterialTask_PickTurntableToTray(index + 1U);

    if ((index + 1U) < ADVANCE_MATERIAL_TASK_ITEM_COUNT)
    {
      AdvanceMaterialTask_PrepareTurntablePickup();
    }
  }

  (void)detect_stop();
}

static void AdvanceMaterialTask_Process(
    const uint8_t positions[ADVANCE_MATERIAL_TASK_ITEM_COUNT])
{
  uint8_t index;

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    AdvanceMaterialTask_PlaceTrayToPosition(index + 1U,
                                            positions[index],
                                            false);
  }

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    AdvanceMaterialTask_PickPositionToTray(positions[index],
                                           index + 1U);
  }
}

static uint8_t AdvanceMaterialTask_FindPosition1(
    const AdvanceMaterialTask_t *task,
    uint8_t color)
{
  uint8_t index;

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    if (task->color1[index] == color)
    {
      return task->position1[index];
    }
  }

  return 0U;
}

bool AdvanceMaterialTask_ParseCode(
    const char code[DETECT_QR_CODE_LENGTH + 1U],
    AdvanceMaterialTask_t *task)
{
  AdvanceMaterialTask_t parsed = {0};
  uint8_t color_mask1 = 0U;
  uint8_t color_mask2 = 0U;
  uint8_t position_mask1 = 0U;
  uint8_t position_mask2 = 0U;
  uint8_t index;

  if ((code == NULL) || (task == NULL))
  {
    return false;
  }

  if ((code[3] != '+') ||
      (code[7] != '+') ||
      (code[11] != '+') ||
      (code[DETECT_QR_CODE_LENGTH] != '\0'))
  {
    return false;
  }

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    uint8_t bit;

    if (!AdvanceMaterialTask_ParseDigit(code[index],
                                        1U,
                                        4U,
                                        &parsed.color1[index]) ||
        !AdvanceMaterialTask_ParseDigit(code[4U + index],
                                        1U,
                                        3U,
                                        &parsed.position1[index]) ||
        !AdvanceMaterialTask_ParseDigit(code[8U + index],
                                        1U,
                                        4U,
                                        &parsed.color2[index]) ||
        !AdvanceMaterialTask_ParseDigit(code[12U + index],
                                        1U,
                                        3U,
                                        &parsed.position2[index]))
    {
      return false;
    }

    bit = (uint8_t)(1U << (parsed.color1[index] - 1U));
    if ((color_mask1 & bit) != 0U)
    {
      return false;
    }
    color_mask1 |= bit;

    bit = (uint8_t)(1U << (parsed.color2[index] - 1U));
    if ((color_mask2 & bit) != 0U)
    {
      return false;
    }
    color_mask2 |= bit;

    bit = (uint8_t)(1U << (parsed.position1[index] - 1U));
    if ((position_mask1 & bit) != 0U)
    {
      return false;
    }
    position_mask1 |= bit;

    bit = (uint8_t)(1U << (parsed.position2[index] - 1U));
    if ((position_mask2 & bit) != 0U)
    {
      return false;
    }
    position_mask2 |= bit;
  }

  if ((color_mask1 != color_mask2) ||
      (position_mask1 != 0x07U) ||
      (position_mask2 != 0x07U))
  {
    return false;
  }

  *task = parsed;
  return true;
}

void AdvanceMaterialTask_Collect1(const AdvanceMaterialTask_t *task)
{
  AdvanceMaterialTask_Collect(task->color1);
}

void AdvanceMaterialTask_Process1(const AdvanceMaterialTask_t *task)
{
  AdvanceMaterialTask_Process(task->position1);
}

void AdvanceMaterialTask_Store1(const AdvanceMaterialTask_t *task)
{
  uint8_t index;

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    AdvanceMaterialTask_PlaceTrayToPosition(index + 1U,
                                            task->position1[index],
                                            false);
  }
}

void AdvanceMaterialTask_Collect2(const AdvanceMaterialTask_t *task)
{
  AdvanceMaterialTask_Collect(task->color2);
}

void AdvanceMaterialTask_Process2(const AdvanceMaterialTask_t *task)
{
  AdvanceMaterialTask_Process(task->position2);
}

void AdvanceMaterialTask_Stack2(const AdvanceMaterialTask_t *task)
{
  uint8_t index;

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    uint8_t target_position =
        AdvanceMaterialTask_FindPosition1(task, task->color2[index]);

    AdvanceMaterialTask_PlaceTrayToPosition(index + 1U,
                                            target_position,
                                            true);
  }
}
