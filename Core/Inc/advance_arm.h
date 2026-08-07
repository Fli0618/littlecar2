#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* 设备 ID。 */
#define ARM_ROTATE_SERVO_ID ((uint8_t)1U) /*!< 机械臂旋转舵机 ID。 */
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U) /*!< 夹爪舵机 ID。 */
#define ARM_MATERIAL_SERVO_ID ((uint8_t)3U) /*!< 物料盘舵机 ID。 */
#define ARM_LIFT_MOTOR_ID ((uint8_t)5U) /*!< 升降轴电机 ID。 */
#define ARM_SLIDE_MOTOR_ID ((uint8_t)6U) /*!< 前后滑台电机 ID。 */

/* ==================== 运动参数 ==================== */

/* 升降轴运动参数：只标定上升方向，下降方向自动取反。 */
#define ARM_LIFT_UP_DIRECTION ((uint8_t)1U) /*!< 升降轴上升方向，需实机确认。 */
#define ARM_LIFT_DOWN_DIRECTION ((uint8_t)(ARM_LIFT_UP_DIRECTION ^ 1U)) /*!< 升降轴下降方向。 */
#define ARM_LIFT_ABSOLUTE_DIRECTION ARM_LIFT_DOWN_DIRECTION /*!< 升降轴绝对坐标正方向。 */
#define ARM_LIFT_SPEED ((uint16_t)6000) /*!< 升降轴运动速度。 */
#define ARM_LIFT_ACC ((uint8_t)3000) /*!< 升降轴加速度。 */
#define ARM_LIFT_POS_MAX ((uint32_t)80000) /*!< 升降轴软件最大安全坐标，需实机标定并预留机械余量。 */

/* 前后滑台运动参数：只标定伸出方向，收回方向自动取反。滑台不使用光电限位。 */
#define ARM_SLIDE_EXTEND_DIRECTION ((uint8_t)0U) /*!< 滑台向外伸出方向，需实机确认。 */
#define ARM_SLIDE_RETRACT_DIRECTION ((uint8_t)(ARM_SLIDE_EXTEND_DIRECTION ^ 1U)) /*!< 滑台向内收回方向。 */
#define ARM_SLIDE_ABSOLUTE_DIRECTION ARM_SLIDE_EXTEND_DIRECTION /*!< 滑台绝对坐标正方向。 */
#define ARM_SLIDE_SPEED ((uint16_t)300U) /*!< 滑台运动速度。 */
#define ARM_SLIDE_ACC ((uint8_t)10U) /*!< 滑台加速度。 */
#define ARM_SLIDE_POS_MAX ((uint32_t)28000U) /*!< 滑台最大允许绝对坐标。 */

/* 舵机运动参数。 */
#define ARM_ROTATE_SPEED ((uint16_t)500U) /*!< 机械臂旋转舵机速度。 */
#define ARM_ROTATE_ACC ((uint16_t)20U) /*!< 机械臂旋转舵机加速度。 */
#define ARM_GRIPPER_SPEED ((uint16_t)500U) /*!< 夹爪舵机速度。 */
#define ARM_GRIPPER_ACC ((uint16_t)20U) /*!< 夹爪舵机加速度。 */
#define ARM_MATERIAL_SPEED ((uint16_t)500U) /*!< 物料盘舵机速度。 */
#define ARM_MATERIAL_ACC ((uint16_t)20U) /*!< 物料盘舵机加速度。 */

/* 升降轴归零参数：仅归零过程读取顶部光电。 */
#define ARM_HOME_SPEED ((uint16_t)80U) /*!< 升降轴寻找零点的速度。 */
#define ARM_HOME_RELEASE_SPEED ((uint16_t)40U) /*!< 升降轴释放顶部限位的速度。 */
#define ARM_HOME_ACC ((uint8_t)10U) /*!< 升降轴归零加速度。 */
#define ARM_HOME_TIMEOUT_MS ((uint32_t)10000U) /*!< 升降轴寻找零点的总超时。 */
#define ARM_HOME_RELEASE_TIMEOUT_MS ((uint32_t)3000U) /*!< 升降轴释放限位超时。 */

/* ==================== 固定点位标定 ==================== */

/* 升降轴固定高度：顶部光电为唯一物理零点，无底部物理限位；所有值须实机标定并位于 0..ARM_LIFT_POS_MAX。 */
#define ARM_LIFT_POS_HOME ((uint32_t)0U) /*!< 升降轴顶部零点。 */
#define ARM_LIFT_POS_LOW ((uint32_t)3200*16) /*!< 通用低位。 */
#define ARM_LIFT_POS_HIGH ((uint32_t)0U) /*!< 通用安全高位。 */
#define ARM_LIFT_POS_PICKUP ((uint32_t)0U) /*!< 外侧原料盘拿取高度。 */
#define ARM_LIFT_POS_TRAY ((uint32_t)0U) /*!< 小车自身物料盘高度。 */
#define ARM_LIFT_POS_STACK ((uint32_t)0U) /*!< 码垛高度。 */

/* 前后滑台固定点位：由人工建立软件零点，不使用光电限位。 */
#define ARM_SLIDE_POS_HOME ((uint32_t)0U) /*!< 滑台人工建立的软件零点。 */
#define ARM_SLIDE_POS_TRAY ((uint32_t)0U) /*!< 滑台移动到小车自身物料盘。 */
#define ARM_SLIDE_POS_PICKUP ((uint32_t)0U) /*!< 滑台移动到外侧原料盘。 */

/* 机械臂旋转舵机固定点位。外侧左、中、右点位均需独立实机标定。 */
#define ARM_ROTATE_POS_OUTWARD_CENTER ((int32_t)0) /*!< 朝向外侧正中心点位。 */
#define ARM_ROTATE_POS_OUTWARD_LEFT ((int32_t)-400) /*!< 朝向外侧左侧点位。 */
#define ARM_ROTATE_POS_OUTWARD_RIGHT ((int32_t)350) /*!< 朝向外侧右侧点位。 */
#define ARM_ROTATE_POS_TRAY ((int32_t)-1650) /*!< 朝向小车自身物料盘。 */

/* 夹爪舵机固定点位。 */
#define ARM_GRIPPER_OPEN_POS ((int32_t)800) /*!< 夹爪张开位置。 */
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800) /*!< 夹爪闭合位置。 */

/* 三槽物料盘固定点位：只标定第一槽位，其余槽位按 4096/圈和 120 度间隔推导。 */
#define ARM_MATERIAL_POSITION_PER_TURN ((int32_t)4096) /*!< 物料盘舵机一圈的位置单位。 */
#define ARM_MATERIAL_POS_1 ((int32_t)1600) /*!< 第一槽位标定位置。 */
#define ARM_MATERIAL_POS_2 \
  ((int32_t)((ARM_MATERIAL_POS_1 + ((ARM_MATERIAL_POSITION_PER_TURN + 1) / 3)) % \
             ARM_MATERIAL_POSITION_PER_TURN)) /*!< 第二槽位，由第一槽位加 120 度得到。 */
#define ARM_MATERIAL_POS_3 \
  ((int32_t)((ARM_MATERIAL_POS_1 + (((ARM_MATERIAL_POSITION_PER_TURN * 2) + 1) / 3)) % \
             ARM_MATERIAL_POSITION_PER_TURN)) /*!< 第三槽位，由第一槽位加 240 度得到。 */

/* 初始化、归零、停止。 */
void AdvanceArm_Init(void);
void AdvanceArm_LiftHomeBlocking(void);
void AdvanceArm_SlideSetCurrentAsZero(void);
void AdvanceArm_Stop(void);
void AdvanceArm_EStop(void);

/* 步进轴基础动作：位置命令下发后仅使用 HAL_Delay 固定等待，不读取电机反馈。 */
void AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse);
void AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse);

/* 夹爪固定动作。 */
void AdvanceArm_GripperOpen(void);
void AdvanceArm_GripperClose(void);

/* 机械臂旋转固定动作。 */
void AdvanceArm_RotateOutwardCenter(void);
void AdvanceArm_RotateOutwardLeft(void);
void AdvanceArm_RotateOutwardRight(void);
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
