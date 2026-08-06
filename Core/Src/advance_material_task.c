#include "advance_material_task.h"

#include <stddef.h>

#include "advance_arm.h"
#include "advance_visual.h"

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

static bool AdvanceMaterialTask_SelectTraySlot(uint8_t slot)
{
  switch (slot)
  {
  case 1U:
    AdvanceArm_TraySlot1();
    return true;

  case 2U:
    AdvanceArm_TraySlot2();
    return true;

  case 3U:
    AdvanceArm_TraySlot3();
    return true;

  default:
    return false;
  }
}

static bool AdvanceMaterialTask_RotateOutward(uint8_t position)
{
  switch (position)
  {
  case 1U:
    AdvanceArm_RotateOutwardLeft();
    return true;

  case 2U:
    AdvanceArm_RotateOutwardCenter();
    return true;

  case 3U:
    AdvanceArm_RotateOutwardRight();
    return true;

  default:
    return false;
  }
}

static bool AdvanceMaterialTask_ToVisualColor(uint8_t color,
                                               ColorType_t *visual_color)
{
  if (visual_color == NULL)
  {
    return false;
  }

  switch (color)
  {
  case 1U:
    *visual_color = RED;
    return true;

  case 2U:
    *visual_color = YELLOW;
    return true;

  case 3U:
    *visual_color = BLUE;
    return true;

  case 4U:
    *visual_color = GREEN;
    return true;

  default:
    return false;
  }
}

static bool AdvanceMaterialTask_PickOutsideToTray(uint8_t position,
                                                   uint8_t tray_slot)
{
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  AdvanceArm_GripperOpen();

  if (!AdvanceMaterialTask_RotateOutward(position))
  {
    return false;
  }

  AdvanceArm_SlideToPickupBlocking();
  AdvanceArm_LiftToPickupBlocking();
  AdvanceArm_GripperClose();
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();

  if (!AdvanceMaterialTask_SelectTraySlot(tray_slot))
  {
    return false;
  }

  AdvanceArm_RotateToTray();
  AdvanceArm_LiftToTrayBlocking();
  AdvanceArm_GripperOpen();
  AdvanceArm_LiftHighBlocking();
  return true;
}

static bool AdvanceMaterialTask_PlaceTrayToOutside(uint8_t tray_slot,
                                                   uint8_t position,
                                                   bool stacking)
{
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();

  if (!AdvanceMaterialTask_SelectTraySlot(tray_slot))
  {
    return false;
  }

  AdvanceArm_RotateToTray();
  AdvanceArm_LiftToTrayBlocking();
  AdvanceArm_GripperClose();
  AdvanceArm_LiftHighBlocking();

  if (!AdvanceMaterialTask_RotateOutward(position))
  {
    return false;
  }

  AdvanceArm_SlideToPickupBlocking();
  if (stacking)
  {
    AdvanceArm_LiftToStackBlocking();
  }
  else
  {
    AdvanceArm_LiftToPickupBlocking();
  }

  AdvanceArm_GripperOpen();
  AdvanceArm_LiftHighBlocking();
  AdvanceArm_SlideToTrayBlocking();
  return true;
}

static bool AdvanceMaterialTask_Collect(
    const uint8_t colors[ADVANCE_MATERIAL_TASK_ITEM_COUNT])
{
  uint8_t index;

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    ColorType_t visual_color;

    if (!AdvanceMaterialTask_ToVisualColor(colors[index], &visual_color))
    {
      return false;
    }

    if (AdvanceVisual_AlignColorBlocking(visual_color) !=
        ADVANCE_VISUAL_STATE_ARRIVED)
    {
      return false;
    }

    if (!AdvanceMaterialTask_PickOutsideToTray(2U, index + 1U))
    {
      return false;
    }
  }

  return true;
}

static bool AdvanceMaterialTask_Process(
    const uint8_t positions[ADVANCE_MATERIAL_TASK_ITEM_COUNT])
{
  uint8_t index;

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    if (!AdvanceMaterialTask_PlaceTrayToOutside(index + 1U,
                                                positions[index],
                                                false))
    {
      return false;
    }
  }

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    if (!AdvanceMaterialTask_PickOutsideToTray(positions[index],
                                               index + 1U))
    {
      return false;
    }
  }

  return true;
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

bool AdvanceMaterialTask_Collect1(const AdvanceMaterialTask_t *task)
{
  return (task != NULL) && AdvanceMaterialTask_Collect(task->color1);
}

bool AdvanceMaterialTask_Process1(const AdvanceMaterialTask_t *task)
{
  return (task != NULL) && AdvanceMaterialTask_Process(task->position1);
}

bool AdvanceMaterialTask_Store1(const AdvanceMaterialTask_t *task)
{
  uint8_t index;

  if (task == NULL)
  {
    return false;
  }

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    if (!AdvanceMaterialTask_PlaceTrayToOutside(index + 1U,
                                                task->position1[index],
                                                false))
    {
      return false;
    }
  }

  return true;
}

bool AdvanceMaterialTask_Collect2(const AdvanceMaterialTask_t *task)
{
  return (task != NULL) && AdvanceMaterialTask_Collect(task->color2);
}

bool AdvanceMaterialTask_Process2(const AdvanceMaterialTask_t *task)
{
  return (task != NULL) && AdvanceMaterialTask_Process(task->position2);
}

bool AdvanceMaterialTask_Stack2(const AdvanceMaterialTask_t *task)
{
  uint8_t index;

  if (task == NULL)
  {
    return false;
  }

  for (index = 0U; index < ADVANCE_MATERIAL_TASK_ITEM_COUNT; ++index)
  {
    uint8_t target_position =
        AdvanceMaterialTask_FindPosition1(task, task->color2[index]);

    if ((target_position == 0U) ||
        !AdvanceMaterialTask_PlaceTrayToOutside(index + 1U,
                                                target_position,
                                                true))
    {
      return false;
    }
  }

  return true;
}
