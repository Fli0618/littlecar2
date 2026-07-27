#ifndef ADVANCE_VISUAL_H
#define ADVANCE_VISUAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "advance_chassis.h"
#include "advance_control.h"
#include "comm_jetson.h"

/* 默认视觉图像尺寸；三组参考点可在后续标定时分别改为独立像素值。 */
#define ADVANCE_VISUAL_IMAGE_WIDTH ((int16_t)640)
#define ADVANCE_VISUAL_IMAGE_HEIGHT ((int16_t)480)

#define ADVANCE_VISUAL_CIRCLE_REF_X ((int16_t)(ADVANCE_VISUAL_IMAGE_WIDTH / 2))
#define ADVANCE_VISUAL_CIRCLE_REF_Y ((int16_t)(ADVANCE_VISUAL_IMAGE_HEIGHT / 2))
#define ADVANCE_VISUAL_COLOR_REF_X ((int16_t)(ADVANCE_VISUAL_IMAGE_WIDTH / 2))
#define ADVANCE_VISUAL_COLOR_REF_Y ((int16_t)(ADVANCE_VISUAL_IMAGE_HEIGHT / 2))
#define ADVANCE_VISUAL_MATERIAL_REF_X ((int16_t)(ADVANCE_VISUAL_IMAGE_WIDTH / 2))
#define ADVANCE_VISUAL_MATERIAL_REF_Y ((int16_t)(ADVANCE_VISUAL_IMAGE_HEIGHT / 2))

#define ADVANCE_VISUAL_KP_X (0.8f)
#define ADVANCE_VISUAL_KP_Y (0.8f)
#define ADVANCE_VISUAL_X_SIGN (1.0f)
#define ADVANCE_VISUAL_Y_SIGN (1.0f)
#define ADVANCE_VISUAL_MAX_VX (100.0f)
#define ADVANCE_VISUAL_MAX_VY (100.0f)
#define ADVANCE_VISUAL_TOLERANCE_X ((int16_t)8)
#define ADVANCE_VISUAL_TOLERANCE_Y ((int16_t)8)
#define ADVANCE_VISUAL_STALE_MS ((uint32_t)120U)
#define ADVANCE_VISUAL_LOST_TIMEOUT_MS ((uint32_t)500U)
#define ADVANCE_VISUAL_TOTAL_TIMEOUT_MS ((uint32_t)5000U)
#define ADVANCE_VISUAL_ARRIVE_COUNT ((uint8_t)3U)
#define ADVANCE_VISUAL_ACC ((uint8_t)5U)

typedef enum
{
  ADVANCE_VISUAL_MODE_CIRCLE = 0U,
  ADVANCE_VISUAL_MODE_COLOR,
  ADVANCE_VISUAL_MODE_MATERIAL
} AdvanceVisual_Mode_t;

typedef enum
{
  ADVANCE_VISUAL_STATE_IDLE = 0U,
  ADVANCE_VISUAL_STATE_RUNNING,
  ADVANCE_VISUAL_STATE_ARRIVED,
  ADVANCE_VISUAL_STATE_TIMEOUT,
  ADVANCE_VISUAL_STATE_NO_TARGET,
  ADVANCE_VISUAL_STATE_START_ERROR,
  ADVANCE_VISUAL_STATE_CANCELED
} AdvanceVisual_State_t;

void AdvanceVisual_Init(void);
void AdvanceVisual_Update(void);

/*
 * 顺序业务可直接调用本接口，例如：
 * (void)AdvanceVisual_AlignBlocking(ADVANCE_VISUAL_MODE_COLOR, material_type);
 * 函数仅等待 TIM6 推进视觉控制并在终态返回。
 */
AdvanceVisual_State_t AdvanceVisual_AlignBlocking(AdvanceVisual_Mode_t mode, uint8_t target_type);

void AdvanceVisual_Cancel(void);
AdvanceVisual_State_t AdvanceVisual_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
