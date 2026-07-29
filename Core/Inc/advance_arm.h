#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

// ID 配置
#define ARM_LIFT_MOTOR_ID ((uint8_t)5U) /*!< 升降轴电机 ID。 */
#define ARM_SLIDE_MOTOR_ID ((uint8_t)6U) /*!< 滑台轴电机 ID。 */
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U) /*!< 夹爪舵机 ID。 */
#define ARM_MATERIAL_SERVO_ID ((uint8_t)3U) /*!< 物料盘舵机 ID。 */

// 升降轴运动配置
#define ARM_LIFT_DOWN_DIRECTION ((uint8_t)0U) /*!< 升降轴下降方向。 */
#define ARM_LIFT_UP_DIRECTION ((uint8_t)1U) /*!< 升降轴上升方向。 */
#define ARM_LIFT_SPEED ((uint16_t)300U) /*!< 升降轴运动速度。 */
#define ARM_LIFT_ACC ((uint8_t)10U) /*!< 升降轴加速度。 */

// 升降轴绝对位置配置
#define ARM_LIFT_ABSOLUTE_DIRECTION ARM_LIFT_DOWN_DIRECTION
#define ARM_LIFT_POS_HOME ((uint32_t)0U)
#define ARM_LIFT_POS_MAX ((uint32_t)18000U)
/* TODO: 以下业务坐标待实机标定，顺序与原预留位置保持一致。 */
#define ARM_LIFT_POS_TRANSITION ((uint32_t)0U)
#define ARM_LIFT_POS_STORAGE ((uint32_t)0U)
#define ARM_LIFT_POS_TURNTABLE ((uint32_t)0U)
#define ARM_LIFT_POS_BUFFER ((uint32_t)0U)
#define ARM_LIFT_POS_BUFFER_LEVEL_2 ((uint32_t)0U)

// 滑台轴运动配置
#define ARM_SLIDE_EXTEND_DIRECTION ((uint8_t)0U) /*!< 滑台轴伸出方向。 */
#define ARM_SLIDE_RETRACT_DIRECTION ((uint8_t)1U) /*!< 滑台轴收回方向。 */
#define ARM_SLIDE_SPEED ((uint16_t)300U) /*!< 滑台轴运动速度。 */
#define ARM_SLIDE_ACC ((uint8_t)10U) /*!< 滑台轴加速度。 */

// 滑台轴绝对位置配置
#define ARM_SLIDE_ABSOLUTE_DIRECTION ARM_SLIDE_EXTEND_DIRECTION
#define ARM_SLIDE_POS_HOME ((uint32_t)0U)
#define ARM_SLIDE_POS_MAX ((uint32_t)28000U)
/* TODO: 以下业务坐标待实机标定，顺序与原预留位置保持一致。 */
#define ARM_SLIDE_POS_TRANSITION ((uint32_t)0U)
#define ARM_SLIDE_POS_STORAGE ((uint32_t)0U)
#define ARM_SLIDE_POS_PICK ((uint32_t)0U)

// 归零与绝对位置控制参数
#define ARM_HOME_SPEED ((uint16_t)80U)
#define ARM_HOME_RELEASE_SPEED ((uint16_t)40U)
#define ARM_HOME_ACC ((uint8_t)10U)
#define ARM_HOME_TIMEOUT_MS ((uint32_t)10000U)
#define ARM_HOME_RELEASE_TIMEOUT_MS ((uint32_t)3000U)
#define ARM_HOME_CONFIRM_MS ((uint32_t)10U)
#define ARM_HOME_COMMAND_DELAY_MS ((uint32_t)100U)

#define ARM_MOVE_TIMEOUT_MS ((uint32_t)5000U)
#define ARM_POSITION_TOLERANCE_PULSE ((int32_t)100)

// 夹爪运动与位置配置
#define ARM_GRIPPER_SPEED ((uint16_t)500U) /*!< 夹爪运动速度。 */
#define ARM_GRIPPER_ACC ((uint16_t)20U) /*!< 夹爪加速度。 */
#define ARM_GRIPPER_OPEN_POS ((int32_t)800) /*!< 夹爪张开目标位置。 */
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800) /*!< 夹爪闭合目标位置。 */
#define ARM_GRIPPER_GuoDu // 中间过度点

// 物料盘运动与位置配置
#define ARM_MATERIAL_SPEED ((uint16_t)500U) /*!< 物料盘旋转速度。 */
#define ARM_MATERIAL_ACC ((uint16_t)20U) /*!< 物料盘旋转加速度。 */
#define ARM_MATERIAL_POS_1 ((int32_t)800)
#define ARM_MATERIAL_POS_2 ((int32_t)800)
#define ARM_MATERIAL_POS_3 ((int32_t)800)

// 机械臂单轴阻塞运动结果
/** @brief 机械臂单轴阻塞运动的明确结果。 */
typedef enum
{
  ADVANCE_ARM_MOVE_OK = 0,
  ADVANCE_ARM_MOVE_LIMIT_REACHED,
  ADVANCE_ARM_MOVE_NOT_HOMED,
  ADVANCE_ARM_MOVE_OUT_OF_RANGE,
  ADVANCE_ARM_MOVE_FEEDBACK_ERROR,
  ADVANCE_ARM_MOVE_MOTOR_FAULT,
  ADVANCE_ARM_MOVE_TIMEOUT
} AdvanceArm_MoveStatus_t;

// 机械臂公共控制接口
/** @brief 初始化归零状态并注册升降轴、滑台轴的反馈监测。 */
void AdvanceArm_Init(void);
/** @brief 依次归零升降轴和滑台轴。 */
bool AdvanceArm_HomeBlocking(void);
/** @brief 查询升降轴归零状态。 */
bool AdvanceArm_IsLiftHomed(void);
/** @brief 查询滑台轴归零状态。 */
bool AdvanceArm_IsSlideHomed(void);
/** @brief 查询两轴是否均已归零。 */
bool AdvanceArm_IsHomed(void);
/** @brief 将升降轴移动到相对零点的绝对脉冲坐标。 */
AdvanceArm_MoveStatus_t AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse);
/** @brief 将滑台轴移动到相对零点的绝对脉冲坐标。 */
AdvanceArm_MoveStatus_t AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse);
/** @brief 控制夹爪张开或闭合。 */
void AdvanceArm_Grab(bool closed);
/** @brief 受控停止两根步进轴，不清除归零状态。 */
void AdvanceArm_Stop(void);
/** @brief 急停两根步进轴并清除归零状态。 */
void AdvanceArm_EStop(void);

#ifdef __cplusplus
}
#endif

#endif
