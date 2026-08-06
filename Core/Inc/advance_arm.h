#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/* 设备 ID。 */
#define ARM_ROTATE_SERVO_ID ((uint8_t)1U) /*!< 机械臂旋转舵机 ID。 */
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U) /*!< 夹爪舵机 ID。 */
#define ARM_MATERIAL_SERVO_ID ((uint8_t)3U) /*!< 物料盘舵机 ID。 */
#define ARM_LIFT_MOTOR_ID ((uint8_t)5U) /*!< 升降轴电机 ID。 */
#define ARM_SLIDE_MOTOR_ID ((uint8_t)6U) /*!< 前后滑台电机 ID。 */

/* 升降轴运动参数。只需标定上升方向，下降方向自动取反。 */
#define ARM_LIFT_UP_DIRECTION ((uint8_t)1U) /*!< 升降轴上升方向，只允许设置为 0U 或 1U。 */
#define ARM_LIFT_DOWN_DIRECTION ((uint8_t)(ARM_LIFT_UP_DIRECTION ^ 1U)) /*!< 升降轴下降方向。 */
#define ARM_LIFT_ABSOLUTE_DIRECTION ARM_LIFT_DOWN_DIRECTION /*!< 升降轴绝对坐标正方向。 */
#define ARM_LIFT_SPEED ((uint16_t)300U) /*!< 升降轴运动速度。 */
#define ARM_LIFT_ACC ((uint8_t)10U) /*!< 升降轴加速度。 */
#define ARM_LIFT_POS_HOME ((uint32_t)0U) /*!< 升降轴顶部零点。 */
#define ARM_LIFT_POS_MAX ((uint32_t)18000U) /*!< 升降轴最大允许绝对坐标。 */
/* TODO: 以下固定高度需根据实机一次性标定后直接修改。 */
#define ARM_LIFT_POS_LOW ((uint32_t)0U) /*!< 通用低位。 */
#define ARM_LIFT_POS_HIGH ((uint32_t)0U) /*!< 通用安全高位。 */
#define ARM_LIFT_POS_PICKUP ((uint32_t)0U) /*!< 外侧原料盘拿取高度。 */
#define ARM_LIFT_POS_TRAY ((uint32_t)0U) /*!< 小车自身物料盘高度。 */
#define ARM_LIFT_POS_STACK ((uint32_t)0U) /*!< 码垛高度。 */

/* 前后滑台运动参数。滑台不使用光电限位，只需标定伸出方向。 */
#define ARM_SLIDE_EXTEND_DIRECTION ((uint8_t)0U) /*!< 滑台向外伸出方向，只允许设置为 0U 或 1U。 */
#define ARM_SLIDE_RETRACT_DIRECTION ((uint8_t)(ARM_SLIDE_EXTEND_DIRECTION ^ 1U)) /*!< 滑台向内收回方向。 */
#define ARM_SLIDE_ABSOLUTE_DIRECTION ARM_SLIDE_EXTEND_DIRECTION /*!< 滑台绝对坐标正方向。 */
#define ARM_SLIDE_SPEED ((uint16_t)300U) /*!< 滑台运动速度。 */
#define ARM_SLIDE_ACC ((uint8_t)10U) /*!< 滑台加速度。 */
#define ARM_SLIDE_POS_HOME ((uint32_t)0U) /*!< 滑台人工建立的零点。 */
#define ARM_SLIDE_POS_MAX ((uint32_t)28000U) /*!< 滑台最大允许绝对坐标。 */
/* TODO: 以下两个固定点位需根据实机一次性标定后直接修改。 */
#define ARM_SLIDE_POS_TRAY ((uint32_t)0U) /*!< 滑台移动到小车自身物料盘。 */
#define ARM_SLIDE_POS_PICKUP ((uint32_t)0U) /*!< 滑台移动到外侧原料盘。 */

/* 升降轴归零参数。 */
#define ARM_HOME_SPEED ((uint16_t)80U) /*!< 升降轴寻找零点的速度。 */
#define ARM_HOME_RELEASE_SPEED ((uint16_t)40U) /*!< 升降轴释放顶部限位的速度。 */
#define ARM_HOME_ACC ((uint8_t)10U) /*!< 升降轴归零加速度。 */
#define ARM_HOME_TIMEOUT_MS ((uint32_t)10000U) /*!< 升降轴寻找零点的总超时。 */
#define ARM_HOME_RELEASE_TIMEOUT_MS ((uint32_t)3000U) /*!< 升降轴释放限位超时。 */
#define ARM_HOME_CONFIRM_MS ((uint32_t)10U) /*!< 升降限位稳定确认时间。 */
#define ARM_HOME_COMMAND_DELAY_MS ((uint32_t)100U) /*!< 停止、清零命令间隔。 */

/* 步进轴阻塞运动保护参数。 */
#define ARM_MOVE_TIMEOUT_MS ((uint32_t)5000U) /*!< 发出位置命令后最多等待 5000 ms，超时则停止电机并退出阻塞函数。 */
#define ARM_POSITION_TOLERANCE_PULSE ((int32_t)100) /*!< 实际位置与目标位置误差不超过 100 脉冲时认为已经到位。 */

/* 机械臂旋转舵机参数。 */
#define ARM_ROTATE_SPEED ((uint16_t)500U)
#define ARM_ROTATE_ACC ((uint16_t)20U)
/* TODO: 以下两个旋转点位需根据实机一次性标定后直接修改。 */
#define ARM_ROTATE_POS_PICKUP ((int32_t)0) /*!< 朝向外侧原料盘。 */
#define ARM_ROTATE_POS_TRAY ((int32_t)0) /*!< 朝向小车自身物料盘。 */

/* 夹爪舵机参数。 */
#define ARM_GRIPPER_SPEED ((uint16_t)500U)
#define ARM_GRIPPER_ACC ((uint16_t)20U)
#define ARM_GRIPPER_OPEN_POS ((int32_t)800) /*!< 夹爪张开位置。 */
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800) /*!< 夹爪闭合位置。 */

/* 三槽物料盘舵机参数。 */
#define ARM_MATERIAL_SPEED ((uint16_t)500U)
#define ARM_MATERIAL_ACC ((uint16_t)20U)
/* TODO: 三个槽位需根据实机一次性标定后直接修改，不设置额外微调量。 */
#define ARM_MATERIAL_POS_1 ((int32_t)800)
#define ARM_MATERIAL_POS_2 ((int32_t)800)
#define ARM_MATERIAL_POS_3 ((int32_t)800)

#define ARM_SERVO_MOVE_DELAY_MS ((uint32_t)1000U) /*!< 舵机动作后的阻塞等待时间。 */

/* 初始化、归零、停止。 */
void AdvanceArm_Init(void);
void AdvanceArm_LiftHomeBlocking(void);
void AdvanceArm_SlideSetCurrentAsZero(void);
void AdvanceArm_Stop(void);
void AdvanceArm_EStop(void);

/* 基础动作接口。 */
void AdvanceArm_Grab(bool closed);
void AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse);
void AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse);

/* 夹爪固定动作。 */
void AdvanceArm_GripperOpen(void);
void AdvanceArm_GripperClose(void);

/* 机械臂旋转固定动作。 */
void AdvanceArm_RotateToPickup(void);
void AdvanceArm_RotateToTray(void);

/* 滑台固定动作。 */
void AdvanceArm_SlideToPickupBlocking(void);
void AdvanceArm_SlideToTrayBlocking(void);

/* 升降轴固定动作。 */
void AdvanceArm_LiftLowBlocking(void);
void AdvanceArm_LiftHighBlocking(void);
void AdvanceArm_LiftToPickupBlocking(void);
void AdvanceArm_LiftToTrayBlocking(void);
void AdvanceArm_LiftToStackBlocking(void);

/* 三槽物料盘固定动作。 */
void AdvanceArm_TraySlot1(void);
void AdvanceArm_TraySlot2(void);
void AdvanceArm_TraySlot3(void);

#ifdef __cplusplus
}
#endif

#endif
