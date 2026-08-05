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
  ADVANCE_CONTROL_VISUAL,     /**< 视觉控制预留模式。 */
  ADVANCE_CONTROL_HOLONOMIC   /**< 轻量全向位置控制器持有控制权。 */
} AdvanceControl_Mode_t;

/** @brief 将控制权初始化为 NONE，不发送底盘命令。 */
void AdvanceControl_Init(void);
/**
 * @brief 设置底盘控制权。
 * @details 切换到 NONE 时立即发送停止命令；两个活动模式之间不能直接切换。
 */
uint8_t AdvanceControl_SetMode(AdvanceControl_Mode_t mode);
/** @brief 仅释放控制权，不附带底盘停车命令。调用方必须先确认停车命令已提交。 */
uint8_t AdvanceControl_ReleaseMode(void);
/** @brief 获取当前唯一的底盘控制权模式。 */
AdvanceControl_Mode_t AdvanceControl_GetMode(void);
/** @brief 查询是否存在活动控制权（mode != NONE）。 */
uint8_t AdvanceControl_IsBusy(void);
/**
 * @brief 取消当前活动控制器并按模式停车。
 * @details WORLD 路由到 AdvanceMotion_Cancel，HOLONOMIC 路由到
 * AdvanceHolonomic_Cancel，其他模式不动作。
 */
void AdvanceControl_CancelActive(void);

#ifdef __cplusplus
}
#endif

#endif
