#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

// ID
#define ARM_LIFT_MOTOR_ID ((uint8_t)5U) /*!< 升降轴电机 ID。 */
#define ARM_SLIDE_MOTOR_ID ((uint8_t)6U) /*!< 滑台轴电机 ID。 */
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U) /*!< 夹爪舵机 ID。 */
#define ARM_MATERIAL_SERVO_ID ((uint8_t)3U) /*!< 物料盘舵机 ID。 */

// 升降轴运动配置
#define ARM_LIFT_DOWN_DIRECTION ((uint8_t)0U) /*!< 升降轴下降方向。 */
#define ARM_LIFT_UP_DIRECTION ((uint8_t)1U) /*!< 升降轴上升方向。 */
#define ARM_LIFT_SPEED ((uint16_t)300U) /*!< 升降轴运动速度。 */
#define ARM_LIFT_ACC ((uint8_t)10U) /*!< 升降轴加速度。 */
// 升降轴位置（应当是相对位置）
#define ARM_LIFT_LOWER_PULSE ((uint32_t)18000U) /*!< 升降轴下降相对脉冲数。 */
#define ARM_LIFT_RAISE_PULSE ((uint32_t)18000U) /*!< 升降轴上升相对脉冲数。 */
#define ARM_LIFT_GuoDu // 中间过度点
#define ARM_LIFT_FangWuLiao // 放物料到车上
#define ARM_LIFT_ZhuanPan // 旋转的物料盘
#define ARM_LIFT_ZanCunQu // 暂存区
#define ARM_LIFT_ZanCunQu2 // 二层物料堆叠

// 前后伸缩运动配置
#define ARM_SLIDE_EXTEND_DIRECTION ((uint8_t)0U) /*!< 滑台轴伸出方向。 */
#define ARM_SLIDE_RETRACT_DIRECTION ((uint8_t)1U) /*!< 滑台轴收回方向。 */
#define ARM_SLIDE_SPEED ((uint16_t)300U) /*!< 滑台轴运动速度。 */
#define ARM_SLIDE_ACC ((uint8_t)10U) /*!< 滑台轴加速度。 */
// 前后伸缩位置
#define ARM_SLIDE_EXTEND_PULSE ((uint32_t)28000U) /*!< 滑台伸出相对脉冲数。 */
#define ARM_SLIDE_RETRACT_PULSE ((uint32_t)28000U) /*!< 滑台收回相对脉冲数。 */
#define ARM_SLIDE_GuoDu // 中甲过度点
#define ARM_SLIDE_FangWuLiao // 放物料到车上
#define ARM_SLIDE_JiaQu // 夹取物料

// 夹爪运动配置
#define ARM_GRIPPER_SPEED ((uint16_t)500U) /*!< 夹爪运动速度。 */
#define ARM_GRIPPER_ACC ((uint16_t)20U) /*!< 夹爪加速度。 */
// 夹爪位置配置
#define ARM_GRIPPER_OPEN_POS ((int32_t)800) /*!< 夹爪张开目标位置。 */
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800) /*!< 夹爪闭合目标位置。 */
#define ARM_GRIPPER_GuoDu // 中间过度点

// 物料盘运动配置
#define ARM_MATERIAL_SPEED ((uint16_t)500U) /*!< 物料盘旋转速度。 */
#define ARM_MATERIAL_ACC ((uint16_t)20U) /*!< 物料盘旋转加速度。 */
// 物料盘位置配置（对应三个盘的位置）
#define ARM_MATERIAL_POS_1 ((int32_t)800)
#define ARM_MATERIAL_POS_2 ((int32_t)800)
#define ARM_MATERIAL_POS_3 ((int32_t)800)


/** @brief 初始化完全开环的机械臂高层模块。 */
void AdvanceArm_Init(void);

/** @brief 控制夹爪打开或闭合，并固定等待 1000 ms。 */
void AdvanceArm_Grab(bool closed);

/** @brief 依次执行伸出、下降、闭合夹爪、上升和收回。 */
void AdvanceArm_Pick(void);

/** @brief 依次执行伸出、下降、打开夹爪、上升和收回。 */
void AdvanceArm_Place(void);

/** @brief 立即停止升降轴与滑台轴。 */
void AdvanceArm_Stop(void);

/** @brief 执行机械臂紧急停止。 */
void AdvanceArm_EStop(void);

#ifdef __cplusplus
}
#endif

#endif
