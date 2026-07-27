#ifndef ADVANCE_CONTROL_H
#define ADVANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "advance_chassis.h"

// 底盘控制器控制权，这个是给视觉控制来用的

typedef enum
{
  ADVANCE_CONTROL_NONE = 0U, /**< 无底盘控制器持有控制权。 */
  ADVANCE_CONTROL_WORLD,      /**< 世界坐标运动控制器持有控制权。 */
  ADVANCE_CONTROL_VISUAL      /**< 视觉控制预留模式。 */
} AdvanceControl_Mode_t;

/** @brief 将控制权初始化为 NONE，不发送底盘命令。 */
void AdvanceControl_Init(void);
/**
 * @brief 设置底盘控制权。
 * @details 切换到 NONE 时立即发送停止命令；两个活动模式之间不能直接切换。
 */
void AdvanceControl_SetMode(AdvanceControl_Mode_t mode);
/** @brief 获取当前唯一的底盘控制权模式。 */
AdvanceControl_Mode_t AdvanceControl_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif
