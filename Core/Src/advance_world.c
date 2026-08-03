#define ADVANCE_WORLD_INTERNAL
#include "advance_world.h"

#include "car_pose.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

#define ADVANCE_WORLD_PI (3.14159265358979323846f)

typedef struct
{
  float raw_x_mm;
  float raw_y_mm;
  float raw_yaw_deg;
  float raw_imu_yaw_deg;
  uint8_t ready;
  uint8_t ops_yaw_ready;
  uint8_t imu_yaw_ready;
} AdvanceWorld_Origin_t;

typedef struct
{
  float x_mm;
  float y_mm;
  float yaw_deg;
} AdvanceWorld_PoseOffset_t;

volatile WorldPose2D_t g_world_pose = {0};
static AdvanceWorld_Origin_t g_world_origin = {0};
static AdvanceWorld_PoseOffset_t g_world_pose_offset = {0};
static volatile AdvanceWorld_Status_t g_world_status = ADVANCE_WORLD_STATUS_NOT_READY;
static volatile AdvanceWorld_YawSource_t g_yaw_source = ADVANCE_WORLD_DEFAULT_YAW_SOURCE;
#if (ADVANCE_WORLD_DEBUG_ENABLE != 0U)
static uint32_t g_last_debug_print_tick = 0U;
#endif

/* 将角度从度转换为弧度。 */
static float AdvanceWorld_DegToRad(float deg)
{
  return deg * ADVANCE_WORLD_PI / 180.0f;
}

/* 根据方向和零偏修正传感器航向角，并归一化到 [-180, 180]。 */
static float AdvanceWorld_NormalizeSensorYawDeg(float yaw_deg, uint8_t reversed, float offset_deg)
{
  if (reversed != 0U)
  {
    yaw_deg = -yaw_deg;
  }

  return AdvanceWorld_WrapAngleDeg(yaw_deg + offset_deg);
}

/* 在极短临界区内复制传感器原始数据，避免读取到中断更新中的半帧。 */
static uint8_t AdvanceWorld_ReadSensorSnapshot(OPS_Pose_t *ops, WIT_Data_t *imu)
{
  uint32_t primask;

  if ((ops == NULL) || (imu == NULL) || (carpose_ops == NULL) || (carpose_imu == NULL))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *ops = *carpose_ops;
  *imu = *carpose_imu;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return 1U;
}

/* 读取并修正 OPS 航向角。 */
static float AdvanceWorld_GetOpsYawDeg(const OPS_Pose_t *ops)
{
  return AdvanceWorld_NormalizeSensorYawDeg(
      ops->zangle_deg,
      ADVANCE_WORLD_OPS_YAW_REVERSED,
      ADVANCE_WORLD_OPS_YAW_OFFSET_DEG);
}

/* 读取并修正 IMU 航向角。 */
static float AdvanceWorld_GetImuYawDeg(const WIT_Data_t *imu)
{
  return AdvanceWorld_NormalizeSensorYawDeg(
      imu->angle_deg.z,
      ADVANCE_WORLD_WIT_YAW_REVERSED,
      ADVANCE_WORLD_WIT_YAW_OFFSET_DEG);
}

/* 判断 OPS 数据是否有效且未超时。 */
static uint8_t AdvanceWorld_IsOpsFresh(const OPS_Pose_t *ops, uint32_t now_tick)
{
  return ((ops != NULL) && (ops->valid != 0U) &&
          ((now_tick - ops->updated_tick) <= ADVANCE_WORLD_SENSOR_TIMEOUT_MS)) ? 1U : 0U;
}

/* 判断 IMU 航向角数据是否有效且未超时。 */
static uint8_t AdvanceWorld_IsImuYawFresh(const WIT_Data_t *imu, uint32_t now_tick)
{
  return ((imu != NULL) && (imu->angle_deg.valid != 0U) &&
          ((now_tick - imu->angle_deg.updated_tick) <= ADVANCE_WORLD_SENSOR_TIMEOUT_MS)) ? 1U : 0U;
}

/* 刷新独立的 OPS/WIT 航向零点；角度 PID 不依赖 OPS 位置原点。 */
static void AdvanceWorld_UpdateYawOrigins(const OPS_Pose_t *ops, const WIT_Data_t *imu, uint32_t now_tick)
{
  if ((g_world_origin.ops_yaw_ready == 0U) && (AdvanceWorld_IsOpsFresh(ops, now_tick) != 0U))
  {
    g_world_origin.raw_yaw_deg = AdvanceWorld_GetOpsYawDeg(ops);
    g_world_origin.ops_yaw_ready = 1U;
  }
  if ((g_world_origin.imu_yaw_ready == 0U) && (AdvanceWorld_IsImuYawFresh(imu, now_tick) != 0U))
  {
    g_world_origin.raw_imu_yaw_deg = AdvanceWorld_GetImuYawDeg(imu);
    g_world_origin.imu_yaw_ready = 1U;
  }
}

static void AdvanceWorld_UpdateRelativeYaws(const OPS_Pose_t *ops, const WIT_Data_t *imu,
                                            const AdvanceWorld_Origin_t *origin,
                                            WorldPose2D_t *pose, uint32_t now_tick)
{
  if ((origin == NULL) || (pose == NULL))
  {
    return;
  }

  if ((AdvanceWorld_IsOpsFresh(ops, now_tick) != 0U) && (origin->ops_yaw_ready != 0U))
  {
    pose->ops_yaw_deg = AdvanceWorld_WrapAngleDeg(AdvanceWorld_GetOpsYawDeg(ops) - origin->raw_yaw_deg);
    pose->ops_yaw_updated_tick = ops->updated_tick;
    pose->ops_yaw_valid = 1U;
  }
  if ((AdvanceWorld_IsImuYawFresh(imu, now_tick) != 0U) && (origin->imu_yaw_ready != 0U))
  {
    pose->wit_yaw_deg = AdvanceWorld_WrapAngleDeg(AdvanceWorld_GetImuYawDeg(imu) - origin->raw_imu_yaw_deg);
    pose->wit_yaw_updated_tick = imu->angle_deg.updated_tick;
    pose->wit_yaw_valid = 1U;
  }
}

static uint8_t AdvanceWorld_SelectYaw(WorldPose2D_t *pose, AdvanceWorld_YawSource_t yaw_source)
{
  if (yaw_source == ADVANCE_WORLD_YAW_SOURCE_OPS)
  {
    if (pose->ops_yaw_valid == 0U) return 0U;
    pose->yaw_deg = pose->ops_yaw_deg;
    pose->yaw_updated_tick = pose->ops_yaw_updated_tick;
    return 1U;
  }
  if (pose->wit_yaw_valid == 0U) return 0U;
  pose->yaw_deg = pose->wit_yaw_deg;
  pose->yaw_updated_tick = pose->wit_yaw_updated_tick;
  return 1U;
}

/* 将 OPS 传感器位置换算为底盘旋转中心位置。 */
static void AdvanceWorld_GetOpsChassisPosition(const OPS_Pose_t *ops, float *x_mm, float *y_mm)
{
  float x = ops->pos_x_mm;
  float y = ops->pos_y_mm;
  float yaw_rad;
  float cos_yaw;
  float sin_yaw;

  if (ADVANCE_WORLD_OPS_XY_SWAPPED != 0U)
  {
    float tmp = x;
    x = y;
    y = tmp;
  }
  if (ADVANCE_WORLD_OPS_X_REVERSED != 0U)
  {
    x = -x;
  }
  if (ADVANCE_WORLD_OPS_Y_REVERSED != 0U)
  {
    y = -y;
  }

  /* OPS 坐标通常是传感器位置，先转换为底盘旋转中心，再建立世界坐标原点。 */
  yaw_rad = AdvanceWorld_DegToRad(AdvanceWorld_GetOpsYawDeg(ops));
  cos_yaw = cosf(yaw_rad);
  sin_yaw = sinf(yaw_rad);
  *x_mm = x - (cos_yaw * ADVANCE_WORLD_OPS_OFFSET_X_MM) + (sin_yaw * ADVANCE_WORLD_OPS_OFFSET_Y_MM);
  *y_mm = y - (sin_yaw * ADVANCE_WORLD_OPS_OFFSET_X_MM) - (cos_yaw * ADVANCE_WORLD_OPS_OFFSET_Y_MM);
}

/* 根据 OPS 和 IMU 反馈更新世界坐标位姿。 */
static AdvanceWorld_Status_t AdvanceWorld_CalculateSensorPose(const OPS_Pose_t *ops, const WIT_Data_t *imu,
                                                               const AdvanceWorld_Origin_t *origin,
                                                               AdvanceWorld_YawSource_t yaw_source,
                                                               uint32_t now_tick, WorldPose2D_t *pose)
{
  float dx_raw;
  float dy_raw;
  float ops_x_mm;
  float ops_y_mm;
  float yaw_rad;
  float cos_yaw;
  float sin_yaw;
  if ((origin == NULL) || (pose == NULL))
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  pose->origin_ready = origin->ready;
  AdvanceWorld_UpdateRelativeYaws(ops, imu, origin, pose, now_tick);

  if (origin->ready == 0U)
  {
    return ADVANCE_WORLD_STATUS_NO_ORIGIN;
  }

  if (AdvanceWorld_IsOpsFresh(ops, now_tick) == 0U)
  {
    return ADVANCE_WORLD_STATUS_NO_OPS;
  }

  AdvanceWorld_GetOpsChassisPosition(ops, &ops_x_mm, &ops_y_mm);
  dx_raw = ops_x_mm - origin->raw_x_mm;
  dy_raw = ops_y_mm - origin->raw_y_mm;
  yaw_rad = AdvanceWorld_DegToRad(origin->raw_yaw_deg);
  cos_yaw = cosf(yaw_rad);
  sin_yaw = sinf(yaw_rad);

  pose->x_mm = cos_yaw * dx_raw + sin_yaw * dy_raw;
  pose->y_mm = -sin_yaw * dx_raw + cos_yaw * dy_raw;
  pose->updated_tick = ops->updated_tick;

  /* 位置时间戳独立保留，完整位姿仍必须依赖当前航向源。 */
  if (AdvanceWorld_SelectYaw(pose, yaw_source) == 0U)
  {
    return ADVANCE_WORLD_STATUS_NO_OPS;
  }

  pose->valid = 1U;
  pose->origin_ready = 1U;
  return ADVANCE_WORLD_STATUS_OK;
}

static void AdvanceWorld_ApplyPoseOffset(WorldPose2D_t *pose)
{
  if ((pose == NULL) || (pose->valid == 0U))
  {
    return;
  }

  pose->x_mm += g_world_pose_offset.x_mm;
  pose->y_mm += g_world_pose_offset.y_mm;
  pose->yaw_deg = AdvanceWorld_WrapAngleDeg(pose->yaw_deg + g_world_pose_offset.yaw_deg);
}

/* 初始化世界坐标、原点和调试输出状态。 */
void AdvanceWorld_Init(void)
{
  g_world_origin = (AdvanceWorld_Origin_t){0};
  g_world_pose_offset = (AdvanceWorld_PoseOffset_t){0};
  g_world_pose = (WorldPose2D_t){0};
  g_world_status = ADVANCE_WORLD_STATUS_NOT_READY;
  g_yaw_source = ADVANCE_WORLD_DEFAULT_YAW_SOURCE;
#if (ADVANCE_WORLD_DEBUG_ENABLE != 0U)
  g_last_debug_print_tick = 0U;
#endif
}

/* 以当前传感器位置和航向建立世界坐标原点。 */
AdvanceWorld_Status_t AdvanceWorld_ResetOrigin(void)
{
  OPS_Pose_t ops;
  WIT_Data_t imu;
  AdvanceWorld_Origin_t candidate_origin = {0};
  AdvanceWorld_PoseOffset_t candidate_offset = {0};
  WorldPose2D_t candidate_pose = {0};
  AdvanceWorld_Status_t status;
  AdvanceWorld_YawSource_t yaw_source;
  uint32_t now_tick;
  uint32_t primask;

  if (AdvanceWorld_ReadSensorSnapshot(&ops, &imu) == 0U)
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  now_tick = HAL_GetTick();
  if (AdvanceWorld_IsOpsFresh(&ops, now_tick) == 0U)
  {
    return ADVANCE_WORLD_STATUS_NO_OPS;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  yaw_source = g_yaw_source;
  if (primask == 0U)
  {
    __enable_irq();
  }

  AdvanceWorld_GetOpsChassisPosition(&ops, &candidate_origin.raw_x_mm, &candidate_origin.raw_y_mm);
  candidate_origin.raw_yaw_deg = AdvanceWorld_GetOpsYawDeg(&ops);
  candidate_origin.ops_yaw_ready = 1U;
  candidate_origin.ready = 1U;
  if (AdvanceWorld_IsImuYawFresh(&imu, now_tick) != 0U)
  {
    candidate_origin.raw_imu_yaw_deg = AdvanceWorld_GetImuYawDeg(&imu);
    candidate_origin.imu_yaw_ready = 1U;
  }
  else if (yaw_source == ADVANCE_WORLD_YAW_SOURCE_WIT)
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  status = AdvanceWorld_CalculateSensorPose(&ops, &imu, &candidate_origin,
                                             yaw_source, now_tick, &candidate_pose);
  if (status != ADVANCE_WORLD_STATUS_OK)
  {
    return status;
  }

  /* 仅提交已完整计算的候选状态，失败重置不会破坏既有世界坐标系。 */
  primask = __get_PRIMASK();
  __disable_irq();
  g_world_origin = candidate_origin;
  g_world_pose_offset = candidate_offset;
  g_world_pose = candidate_pose;
  g_world_status = ADVANCE_WORLD_STATUS_OK;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return ADVANCE_WORLD_STATUS_OK;
}

AdvanceWorld_Status_t AdvanceWorld_PoseOffset(float x_mm, float y_mm, float yaw_deg)
{
  OPS_Pose_t ops;
  WIT_Data_t imu;
  WorldPose2D_t sensor_pose = {0};
  AdvanceWorld_Status_t status;

  if ((isfinite(x_mm) == 0) || (isfinite(y_mm) == 0) || (isfinite(yaw_deg) == 0))
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  sensor_pose.origin_ready = g_world_origin.ready;
  if (AdvanceWorld_ReadSensorSnapshot(&ops, &imu) == 0U)
  {
    g_world_pose = sensor_pose;
    g_world_status = ADVANCE_WORLD_STATUS_NOT_READY;
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  {
    uint32_t now_tick = HAL_GetTick();

    AdvanceWorld_UpdateYawOrigins(&ops, &imu, now_tick);
    status = AdvanceWorld_CalculateSensorPose(&ops, &imu, &g_world_origin,
                                               g_yaw_source, now_tick, &sensor_pose);
  }
  if (status != ADVANCE_WORLD_STATUS_OK)
  {
    g_world_pose = sensor_pose;
    g_world_status = status;
    return status;
  }

  g_world_pose_offset.x_mm = x_mm - sensor_pose.x_mm;
  g_world_pose_offset.y_mm = y_mm - sensor_pose.y_mm;
  g_world_pose_offset.yaw_deg = AdvanceWorld_WrapAngleDeg(yaw_deg - sensor_pose.yaw_deg);

  AdvanceWorld_ApplyPoseOffset(&sensor_pose);
  g_world_pose = sensor_pose;
  g_world_status = ADVANCE_WORLD_STATUS_OK;
  return ADVANCE_WORLD_STATUS_OK;
}

/* 周期性刷新世界坐标位姿。 */
void AdvanceWorld_Update(void)
{
  OPS_Pose_t ops;
  WIT_Data_t imu;
  WorldPose2D_t latest_pose = {0};
  AdvanceWorld_Status_t status;
  uint32_t now_tick;

  latest_pose.origin_ready = g_world_origin.ready;
  if (AdvanceWorld_ReadSensorSnapshot(&ops, &imu) == 0U)
  {
    g_world_pose = latest_pose;
    g_world_status = ADVANCE_WORLD_STATUS_NOT_READY;
    return;
  }

  now_tick = HAL_GetTick();
  AdvanceWorld_UpdateYawOrigins(&ops, &imu, now_tick);
  status = AdvanceWorld_CalculateSensorPose(&ops, &imu, &g_world_origin,
                                             g_yaw_source, now_tick, &latest_pose);
  if (status == ADVANCE_WORLD_STATUS_OK)
  {
    AdvanceWorld_ApplyPoseOffset(&latest_pose);
  }

  g_world_pose = latest_pose;
  g_world_status = status;
}

/* 在极短临界区内复制由 TIM6 更新的世界位姿缓存。 */
AdvanceWorld_Status_t AdvanceWorld_GetPoseCopy(WorldPose2D_t *pose)
{
  uint32_t primask;
  AdvanceWorld_Status_t status;

  if (pose == NULL)
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *pose = g_world_pose;
  status = g_world_status;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return status;
}

AdvanceWorld_Status_t AdvanceWorld_SetYawSource(AdvanceWorld_YawSource_t source)
{
  if ((source != ADVANCE_WORLD_YAW_SOURCE_WIT) && (source != ADVANCE_WORLD_YAW_SOURCE_OPS))
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }
  g_yaw_source = source;
  return ADVANCE_WORLD_STATUS_OK;
}

AdvanceWorld_YawSource_t AdvanceWorld_GetYawSource(void)
{
  return g_yaw_source;
}

AdvanceWorld_Status_t AdvanceWorld_GetYawCopy(float *yaw_deg, uint32_t *updated_tick)
{
  WorldPose2D_t pose = {0};
  OPS_Pose_t ops;
  WIT_Data_t imu;
  uint32_t now_tick = HAL_GetTick();

  if ((yaw_deg == NULL) || (updated_tick == NULL) || (AdvanceWorld_ReadSensorSnapshot(&ops, &imu) == 0U))
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }
  AdvanceWorld_UpdateYawOrigins(&ops, &imu, now_tick);
  AdvanceWorld_UpdateRelativeYaws(&ops, &imu, &g_world_origin, &pose, now_tick);
  if (AdvanceWorld_SelectYaw(&pose, g_yaw_source) == 0U)
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }
  *yaw_deg = pose.yaw_deg;
  *updated_tick = pose.yaw_updated_tick;
  return ADVANCE_WORLD_STATUS_OK;
}

/* 将角度归一化到 [-180, 180] 度范围。 */
float AdvanceWorld_WrapAngleDeg(float angle_deg)
{
  angle_deg = fmodf(angle_deg, 360.0f);
  if (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  else if (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }

  return angle_deg;
}

/* 将浮点数限制在指定最小值和最大值之间。 */
float AdvanceWorld_LimitFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/* 将世界坐标系线速度转换为车体坐标系线速度。 */
void AdvanceWorld_WorldToBodyVelocity(float vx_w, float vy_w, float yaw_deg, float *vx_b, float *vy_b)
{
  float yaw = AdvanceWorld_DegToRad(yaw_deg);
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);

  if ((vx_b == NULL) || (vy_b == NULL))
  {
    return;
  }

  *vx_b = cos_yaw * vx_w + sin_yaw * vy_w;
  *vy_b = -sin_yaw * vx_w + cos_yaw * vy_w;
}

/* 将车体坐标系线速度转换为世界坐标系线速度。 */
void AdvanceWorld_BodyToWorldVelocity(float vx_b, float vy_b, float yaw_deg, float *vx_w, float *vy_w)
{
  float yaw = AdvanceWorld_DegToRad(yaw_deg);
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);

  if ((vx_w == NULL) || (vy_w == NULL))
  {
    return;
  }

  *vx_w = cos_yaw * vx_b - sin_yaw * vy_b;
  *vy_w = sin_yaw * vx_b + cos_yaw * vy_b;
}

/* 按配置周期输出传感器原始数据和世界位姿调试信息。 */
void AdvanceWorld_PrintDebug(void)
{
#if (ADVANCE_WORLD_DEBUG_ENABLE != 0U)
  uint32_t now_tick = HAL_GetTick();

  if ((now_tick - g_last_debug_print_tick) < ADVANCE_WORLD_DEBUG_PRINT_MS)
  {
    return;
  }
  g_last_debug_print_tick = now_tick;

  if (carpose_ops != NULL)
  {
    printf("OPS_RAW: valid=%u x_mm=%ld y_mm=%ld yaw_cdeg=%ld\r\n",
           carpose_ops->valid,
           (long)carpose_ops->pos_x_mm,
           (long)carpose_ops->pos_y_mm,
           (long)(carpose_ops->zangle_deg * 100.0f));
  }

  if (carpose_imu != NULL)
  {
    printf("WIT_RAW: angle_valid=%u yaw_cdeg=%ld origin=%u\r\n",
           carpose_imu->angle_deg.valid,
           (long)(carpose_imu->angle_deg.z * 100.0f),
           g_world_origin.imu_yaw_ready);
  }

  printf("POSE_WORLD: valid=%u origin=%u x_mm=%ld y_mm=%ld yaw_cdeg=%ld\r\n",
         g_world_pose.valid,
         g_world_pose.origin_ready,
         (long)g_world_pose.x_mm,
         (long)g_world_pose.y_mm,
         (long)(g_world_pose.yaw_deg * 100.0f));
#endif
}
