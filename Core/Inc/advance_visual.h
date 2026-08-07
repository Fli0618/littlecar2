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

/* 三类任务的目标参考点，均处于 Jetson 原始 640x480 像素坐标系。 */
#define ADVANCE_VISUAL_CIRCLE_REF_X ((int16_t)(ADVANCE_VISUAL_IMAGE_WIDTH / 2))
#define ADVANCE_VISUAL_CIRCLE_REF_Y ((int16_t)(ADVANCE_VISUAL_IMAGE_HEIGHT / 2))
#define ADVANCE_VISUAL_COLOR_REF_X ((int16_t)(ADVANCE_VISUAL_IMAGE_WIDTH / 2))
#define ADVANCE_VISUAL_COLOR_REF_Y ((int16_t)(ADVANCE_VISUAL_IMAGE_HEIGHT / 2))
#define ADVANCE_VISUAL_DISK_CENTER_REF_X ((int16_t)(ADVANCE_VISUAL_IMAGE_WIDTH / 2))
#define ADVANCE_VISUAL_DISK_CENTER_REF_Y ((int16_t)(ADVANCE_VISUAL_IMAGE_HEIGHT / 2))

/* 像素误差到车体横向/前向速度的比例系数；需结合实车低速标定调整。 */
#define ADVANCE_VISUAL_KP_X (0.8f)
#define ADVANCE_VISUAL_KP_Y (1.05f)
/* 相机旋转映射后的车体轴方向修正；只能取 1.0f 或 -1.0f，不能替代旋转配置。 */
#define ADVANCE_VISUAL_BODY_X_SIGN (-1.0f)
#define ADVANCE_VISUAL_BODY_Y_SIGN (1.0f)
/* 视觉伺服允许下发的横向和前向最大速度，单位 mm/s。 */
#define ADVANCE_VISUAL_MAX_VX (100.0f)
#define ADVANCE_VISUAL_MAX_VY (100.0f)
/* 车体轴误差的到达死区，单位为映射后的像素误差。 */
#define ADVANCE_VISUAL_TOLERANCE_X ((int16_t)2)
#define ADVANCE_VISUAL_TOLERANCE_Y ((int16_t)2)
/* 视觉帧、目标丢失和总任务超时，单位 ms。 */
#define ADVANCE_VISUAL_STALE_MS ((uint32_t)120U)
#define ADVANCE_VISUAL_LOST_TIMEOUT_MS ((uint32_t)1500U)
#define ADVANCE_VISUAL_TOTAL_TIMEOUT_MS ((uint32_t)10000U)
#define ADVANCE_VISUAL_TARGET_LOCK_MAX_JUMP_PX ((int16_t)120)
#define ADVANCE_VISUAL_TARGET_DISTANCE_TIE_PX ((int16_t)8)
#define ADVANCE_VISUAL_ARRIVE_COUNT ((uint8_t)3U)
#define ADVANCE_VISUAL_ACC ((uint8_t)5U)

/* 阻塞式视觉控制步骤和诊断日志的独立周期，单位为 ms。 */
#define ADVANCE_VISUAL_CONTROL_PERIOD_MS ((uint32_t)20U)
#define ADVANCE_VISUAL_LOG_PERIOD_MS ((uint32_t)200U)

#ifndef ADVANCE_VISUAL_DEBUG_LOG_ENABLE
#define ADVANCE_VISUAL_DEBUG_LOG_ENABLE (0U)
#endif

typedef enum
{
  RED = 0U,
  YELLOW,
  BLUE,
  GREEN,
  BLACK,
  LIGHT_BLUE,
  EMPTY_SLOT
} ColorType_t;

typedef enum
{
  NUMBER_1 = 0U,
  NUMBER_2,
  NUMBER_3
} CircleType_t;

/* 相机图像相对车体坐标的顺时针安装角度。 */
typedef enum
{
  ADVANCE_VISUAL_CAMERA_ROTATION_0 = 0U,
  ADVANCE_VISUAL_CAMERA_ROTATION_90_CW,
  ADVANCE_VISUAL_CAMERA_ROTATION_180,
  ADVANCE_VISUAL_CAMERA_ROTATION_270_CW
} AdvanceVisual_CameraRotation_t;

/* 默认值对应当前相机安装方向；仅在编译期按实际安装角度修改。 */
#ifndef ADVANCE_VISUAL_CAMERA_ROTATION
#define ADVANCE_VISUAL_CAMERA_ROTATION ADVANCE_VISUAL_CAMERA_ROTATION_90_CW
#endif

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

/*
 * 顺序业务可直接调用以下阻塞接口；接口内部按固定周期读取 Jetson
 * 结果并推进视觉伺服，不依赖 TIM6 调用视觉更新函数。UART DMA、SysTick
 * 和底层驱动中断仍需保持运行。
 */
AdvanceVisual_State_t AdvanceVisual_AlignColorBlocking(ColorType_t color);
AdvanceVisual_State_t AdvanceVisual_AlignDiskCenterBlocking(void);
AdvanceVisual_State_t AdvanceVisual_AlignCircleBlocking(CircleType_t number);

void AdvanceVisual_Cancel(void);
AdvanceVisual_State_t AdvanceVisual_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
