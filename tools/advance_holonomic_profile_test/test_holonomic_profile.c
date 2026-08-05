/*
 * 全向位置控制器（advance_holonomic_position）主机侧单元测试。
 *
 * 仅由 build_and_run.ps1 以 -DADVANCE_HOLONOMIC_UNIT_TEST 编译运行，
 * 不加入 EIDE/MDK 固件工程，也不进入 TIM6 正常控制路径。
 * 通过直接包含被测 .c 的方式调用其内部 static 轮廓函数，
 * 并用本文件提供的桩替代 HAL/底盘/世界/控制权依赖。
 */
#if defined(ADVANCE_HOLONOMIC_UNIT_TEST)

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* 真实头：位姿/目标类型；桩头：控制权枚举与函数声明 */
#include "advance_holonomic_position.h"
#include "advance_control.h"

/* ARM 中断原语桩（主机无 CMSIS） */
static inline uint32_t __get_PRIMASK(void)
{
  return 0U;
}
static inline void __disable_irq(void)
{
}
static inline void __enable_irq(void)
{
}
static inline void __WFI(void)
{
}

/* 可直接注入的测试状态 */
static uint32_t g_test_tick = 0U;
static uint32_t g_chassis_calls = 0U;
static WorldPose2D_t g_test_pose = {0};
static float g_test_yaw_deg = 0.0f;
static uint32_t g_test_yaw_updated_tick = 0U;
static AdvanceControl_Mode_t g_test_mode = ADVANCE_CONTROL_NONE;

/* 直接包含被测模块，使内部 static 符号对本翻译单元可见 */
#include "../../Core/Src/advance_holonomic_position.c"

/* ---------------- 外部依赖桩 ---------------- */

uint32_t HAL_GetTick(void)
{
  return g_test_tick;
}

uint8_t Chassis_SetBodyVelocityEx(float vx_right_mm_s, float vy_forward_mm_s,
                                  float wz_ccw_deg_s, uint8_t acc)
{
  (void)vx_right_mm_s;
  (void)vy_forward_mm_s;
  (void)wz_ccw_deg_s;
  (void)acc;
  ++g_chassis_calls;
  return 1U;
}

uint8_t Chassis_Stop(void)
{
  ++g_chassis_calls;
  return 1U;
}

uint8_t Chassis_SmoothStop(uint8_t acc)
{
  (void)acc;
  ++g_chassis_calls;
  return 1U;
}

AdvanceControl_Mode_t AdvanceControl_GetMode(void)
{
  return g_test_mode;
}

uint8_t AdvanceControl_SetMode(AdvanceControl_Mode_t mode)
{
  if ((mode != ADVANCE_CONTROL_NONE) &&
      (mode != ADVANCE_CONTROL_WORLD) &&
      (mode != ADVANCE_CONTROL_VISUAL) &&
      (mode != ADVANCE_CONTROL_HOLONOMIC))
  {
    return 0U;
  }
  if ((mode != ADVANCE_CONTROL_NONE) &&
      (g_test_mode != ADVANCE_CONTROL_NONE) &&
      (g_test_mode != mode))
  {
    return 0U;
  }
  g_test_mode = mode;
  return 1U;
}

uint8_t AdvanceControl_ReleaseMode(void)
{
  g_test_mode = ADVANCE_CONTROL_NONE;
  return 1U;
}

void AdvanceControl_CancelActive(void)
{
  if (g_test_mode == ADVANCE_CONTROL_HOLONOMIC)
  {
    AdvanceHolonomic_Cancel();
  }
}

AdvanceWorld_Status_t AdvanceWorld_GetPoseCopy(WorldPose2D_t *pose)
{
  if (pose == NULL)
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }
  *pose = g_test_pose;
  return (g_test_pose.origin_ready != 0U) ? ADVANCE_WORLD_STATUS_OK
                                          : ADVANCE_WORLD_STATUS_NO_ORIGIN;
}

AdvanceWorld_Status_t AdvanceWorld_GetYawCopy(float *yaw_deg, uint32_t *updated_tick)
{
  if ((yaw_deg == NULL) || (updated_tick == NULL))
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }
  *yaw_deg = g_test_yaw_deg;
  *updated_tick = g_test_yaw_updated_tick;
  return ADVANCE_WORLD_STATUS_OK;
}

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

void AdvanceWorld_WorldToBodyVelocity(float vx_w, float vy_w, float yaw_deg,
                                      float *vx_b, float *vy_b)
{
  float rad = yaw_deg * 3.14159265358979323846f / 180.0f;
  float cos_yaw = cosf(rad);
  float sin_yaw = sinf(rad);

  if ((vx_b == NULL) || (vy_b == NULL))
  {
    return;
  }
  *vx_b = (cos_yaw * vx_w) + (sin_yaw * vy_w);
  *vy_b = (-sin_yaw * vx_w) + (cos_yaw * vy_w);
}

void AdvanceWorld_BodyToWorldVelocity(float vx_b, float vy_b, float yaw_deg,
                                      float *vx_w, float *vy_w)
{
  float rad = yaw_deg * 3.14159265358979323846f / 180.0f;
  float cos_yaw = cosf(rad);
  float sin_yaw = sinf(rad);

  if ((vx_w == NULL) || (vy_w == NULL))
  {
    return;
  }
  *vx_w = (cos_yaw * vx_b) - (sin_yaw * vy_b);
  *vy_w = (sin_yaw * vx_b) + (cos_yaw * vy_b);
}

/* ---------------- 断言工具 ---------------- */

static int g_failures = 0;

static void Check(int ok, const char *message)
{
  if (ok != 0)
  {
    printf("PASS: %s\n", message);
  }
  else
  {
    printf("FAIL: %s\n", message);
    ++g_failures;
  }
}

static int FloatNear(float a, float b, float tolerance)
{
  return (fabsf(a - b) <= tolerance) ? 1 : 0;
}

/* 以固定步长将轮廓推进到 finished */
static void RunProfileToEnd(AdvanceHolonomic_Profile1D_t *profile, float dt_s)
{
  uint32_t guard = 0U;

  while ((profile->finished == 0U) && (guard < 100000U))
  {
    AdvanceHolonomic_Profile1D_Update(profile, dt_s);
    ++guard;
  }
}

/* ---------------- 解析轮廓测试 ---------------- */

static void TestProfileTrapezoid(float distance, float max_velocity,
                                 float acceleration, float deceleration,
                                 const char *name)
{
  AdvanceHolonomic_Profile1D_t p;
  float prev_position = 0.0f;
  float peak_expected;
  uint32_t guard = 0U;
  uint8_t monotonic = 1U;

  AdvanceHolonomic_Profile1D_Init(&p, distance, max_velocity, acceleration, deceleration);
  Check(FloatNear(p.position, 0.0f, 0.0f), "初始 position = 0");
  Check(FloatNear(p.velocity, 0.0f, 0.0f), "初始 velocity = 0");
  Check(FloatNear(p.elapsed_time_s, 0.0f, 0.0f), "初始 elapsed_time_s = 0");
  Check(p.finished == 0U, "非零距离初始未完成");
  Check(p.direction == 1.0f, "正目标 direction = +1");

  peak_expected = (p.accel_distance > 0.0f) ? (p.acceleration * p.accel_time_s) : 0.0f;
  Check(FloatNear(p.accel_distance,
                  0.5f * acceleration * p.accel_time_s * p.accel_time_s, 1e-3f),
        "加速段终点位置连续");
  Check(FloatNear(peak_expected, p.peak_velocity, 1e-3f), "加速段终点速度连续");
  if (p.cruise_time_s > 0.0f)
  {
    Check(FloatNear(p.accel_distance + (p.peak_velocity * p.cruise_time_s),
                    p.accel_distance + p.cruise_distance, 1e-3f),
          "巡航段终点位置连续");
    Check(p.cruise_distance > 0.0f, "梯形轮廓存在巡航段");
  }
  else
  {
    Check(p.cruise_distance == 0.0f, "三角形轮廓无巡航段");
  }
  Check(FloatNear(p.total_time_s,
                  p.accel_time_s + p.cruise_time_s + p.decel_time_s, 1e-6f),
        "总时间 = 三段之和");

  while ((p.finished == 0U) && (guard < 100000U))
  {
    if (p.position < prev_position)
    {
      monotonic = 0U;
    }
    prev_position = p.position;
    AdvanceHolonomic_Profile1D_Update(&p, 0.01f);
    ++guard;
  }
  Check(p.finished == 1U, "轮廓结束时 finished = 1");
  Check(FloatNear(p.position, distance, 1e-3f), "最终 position = goal");
  Check(FloatNear(p.velocity, 0.0f, 1e-4f), "最终 velocity = 0");
  Check(monotonic != 0U, "正目标位置单调不减");

  printf("[PROFILE] %s (%.0f mm) peak=%.2f total=%.3fs done\n",
         name, distance, p.peak_velocity, p.total_time_s);
}

static void TestProfileNegative(void)
{
  AdvanceHolonomic_Profile1D_t p;
  float max_velocity_during = 0.0f;
  uint32_t guard = 0U;

  AdvanceHolonomic_Profile1D_Init(&p, -100.0f, 300.0f, 600.0f, 800.0f);
  Check(p.direction == -1.0f, "负目标 direction = -1");
  while ((p.finished == 0U) && (guard < 100000U))
  {
    if (p.velocity > max_velocity_during)
    {
      max_velocity_during = p.velocity;
    }
    AdvanceHolonomic_Profile1D_Update(&p, 0.01f);
    ++guard;
  }
  Check(max_velocity_during <= 1e-4f, "负目标运动期间速度符号为负");
  Check(p.finished == 1U, "负目标轮廓完成");
  Check(FloatNear(p.position, -100.0f, 1e-3f), "负目标最终 position = goal");
  Check(FloatNear(p.velocity, 0.0f, 1e-4f), "负目标最终 velocity = 0");
}

static void TestProfileZeroDistance(void)
{
  AdvanceHolonomic_Profile1D_t p;

  AdvanceHolonomic_Profile1D_Init(&p, 0.0f, 300.0f, 600.0f, 800.0f);
  Check(p.finished == 1U, "零距离目标直接完成");
  Check(FloatNear(p.position, 0.0f, 0.0f), "零距离 position = 0");
  Check(FloatNear(p.velocity, 0.0f, 0.0f), "零距离 velocity = 0");
  AdvanceHolonomic_Profile1D_Update(&p, 0.01f);
  Check(p.finished == 1U, "完成后 Update 不再推进");
  Check(FloatNear(p.position, 0.0f, 0.0f), "完成后 position 不变");
}

static void TestProfileYaw(void)
{
  AdvanceHolonomic_Profile1D_t p;

  AdvanceHolonomic_Profile1D_Init(&p, 30.0f, 45.0f, 150.0f, 150.0f);
  RunProfileToEnd(&p, 0.01f);
  Check(FloatNear(p.position, 30.0f, 1e-3f), "航向 30° 最终 position = goal");
  Check(FloatNear(p.velocity, 0.0f, 1e-4f), "航向 30° 最终 velocity = 0");

  AdvanceHolonomic_Profile1D_Init(&p, 90.0f, 45.0f, 150.0f, 150.0f);
  RunProfileToEnd(&p, 0.01f);
  Check(FloatNear(p.position, 90.0f, 1e-3f), "航向 90° 最终 position = goal");

  AdvanceHolonomic_Profile1D_Init(&p, -90.0f, 45.0f, 150.0f, 150.0f);
  RunProfileToEnd(&p, 0.01f);
  Check(FloatNear(p.position, -90.0f, 1e-3f), "航向 -90° 最终 position = goal");
  Check(p.direction == -1.0f, "负航向 direction = -1");

  /* 180° 附近最短有符号角度：start=170° → goal=-170° 应规划 +20° */
  Check(FloatNear(AdvanceWorld_WrapAngleDeg(-170.0f - 170.0f), 20.0f, 1e-4f),
        "±180° 附近最短角度差 = +20°");
  AdvanceHolonomic_Profile1D_Init(&p, 20.0f, 45.0f, 150.0f, 150.0f);
  RunProfileToEnd(&p, 0.01f);
  Check(FloatNear(p.position, 20.0f, 1e-3f), "最短角轮廓最终 position = +20°");
}

/* ---------------- dt == 0 与状态/快照测试 ---------------- */

static void SetupFreshPose(float x_mm, float y_mm, float yaw_deg)
{
  g_test_pose = (WorldPose2D_t){0};
  g_test_pose.origin_ready = 1U;
  g_test_pose.valid = 1U;
  g_test_pose.x_mm = x_mm;
  g_test_pose.y_mm = y_mm;
  g_test_pose.yaw_deg = yaw_deg;
  g_test_pose.updated_tick = g_test_tick;
  g_test_pose.yaw_updated_tick = g_test_tick;
  g_test_yaw_deg = yaw_deg;
  g_test_yaw_updated_tick = g_test_tick;
}

static WorldGoalPose2D_t MakeForwardGoal(float x_mm, float y_mm, float yaw_deg)
{
  WorldGoalPose2D_t goal = {0};

  goal.x_mm = x_mm;
  goal.y_mm = y_mm;
  goal.yaw_deg = yaw_deg;
  goal.vmax_mm_s = 300.0f;
  goal.wmax_deg_s = 60.0f;
  goal.timeout_ms = 8000U;
  goal.goal_flags = ADVANCE_HOLONOMIC_GOAL_USE_POSITION | ADVANCE_HOLONOMIC_GOAL_USE_YAW;
  return goal;
}

static void TestDtZeroAndSnapshotClear(void)
{
  WorldGoalPose2D_t goal;
  float pos_before;
  float vel_before;
  float elapsed_before;
  uint32_t tick_before;
  uint32_t calls_before;
  int i;

  g_test_tick = 1000U;
  SetupFreshPose(0.0f, 0.0f, 0.0f);
  goal = MakeForwardGoal(0.0f, 300.0f, 0.0f);

  AdvanceHolonomic_Init();
  Check(AdvanceHolonomic_Start(&goal, 30U) == ADVANCE_HOLONOMIC_STATUS_OK, "Start 成功");
  Check(AdvanceHolonomic_IsActive() == 1U, "启动后处于活动状态");
  Check(g_holonomic_state == ADVANCE_HOLONOMIC_STATE_RUNNING, "volatile 状态为 RUNNING");

  pos_before = g_holonomic.linear_profile.position;
  vel_before = g_holonomic.linear_profile.velocity;
  elapsed_before = g_holonomic.linear_profile.elapsed_time_s;
  tick_before = g_holonomic.last_update_tick;
  calls_before = g_chassis_calls;

  /* now_tick == last_update_tick：必须直接返回 */
  AdvanceHolonomic_Update();
  Check(g_holonomic.linear_profile.position == pos_before, "dt=0 不推进 position");
  Check(g_holonomic.linear_profile.velocity == vel_before, "dt=0 不改变 velocity");
  Check(g_holonomic.linear_profile.elapsed_time_s == elapsed_before, "dt=0 不推进 elapsed");
  Check(g_holonomic.last_update_tick == tick_before, "dt=0 不更新 last_update_tick");
  Check(g_chassis_calls == calls_before, "dt=0 不下发底盘命令");

  /* 推进若干正常控制周期，使调试修正/命令字段非零 */
  for (i = 0; i < 5; ++i)
  {
    g_test_tick += 20U;
    SetupFreshPose(0.0f, 0.0f, 0.0f);
    AdvanceHolonomic_Update();
  }
  Check(g_holonomic.debug.command_forward_mm_s != 0.0f, "第一任务产生前向命令");
  Check(g_chassis_calls > calls_before, "正常周期下发底盘命令");

  AdvanceHolonomic_Cancel();
  Check(g_holonomic_state == ADVANCE_HOLONOMIC_STATE_CANCELED, "Cancel 进入 CANCELED");
  Check(AdvanceControl_GetMode() == ADVANCE_CONTROL_NONE, "Cancel 后释放控制权");

  /* 第二次 Start：首个控制周期前调试字段必须清零 */
  g_test_tick += 20U;
  SetupFreshPose(0.0f, 0.0f, 0.0f);
  goal = MakeForwardGoal(0.0f, 300.0f, 0.0f);
  Check(AdvanceHolonomic_Start(&goal, 30U) == ADVANCE_HOLONOMIC_STATUS_OK, "第二次 Start 成功");
  Check(g_holonomic.debug.position_correction_forward_mm_s == 0.0f, "新任务位置修正清零");
  Check(g_holonomic.debug.position_correction_lateral_mm_s == 0.0f, "新任务横向修正清零");
  Check(g_holonomic.debug.velocity_correction_forward_mm_s == 0.0f, "新任务速度修正清零");
  Check(g_holonomic.debug.command_forward_mm_s == 0.0f, "新任务前向命令清零");
  Check(g_holonomic.debug.command_lateral_mm_s == 0.0f, "新任务横向命令清零");
  Check(g_holonomic.debug.command_wz_deg_s == 0.0f, "新任务航向命令清零");
  Check(g_holonomic.debug.drive_forward_mm_s == 0.0f, "新任务校准前向清零");
  Check(g_holonomic.debug.drive_lateral_mm_s == 0.0f, "新任务校准横向清零");
  Check(g_holonomic.debug.drive_wz_deg_s == 0.0f, "新任务校准航向清零");
  Check(FloatNear(g_holonomic.debug.reference_x_mm, g_holonomic.start_x_mm, 1e-6f),
        "新任务参考 X 初值 = 起点");
  Check(FloatNear(g_holonomic.debug.reference_y_mm, g_holonomic.start_y_mm, 1e-6f),
        "新任务参考 Y 初值 = 起点");
  Check(g_holonomic.debug.state == ADVANCE_HOLONOMIC_STATE_RUNNING, "新任务快照状态 = RUNNING");
}

/* 测试入口依赖的车体到世界位移语义：body(0,300) 在任意航向下都指向车体前方 */
static void TestBodyToWorldForward(void)
{
  float dx;
  float dy;

  AdvanceWorld_BodyToWorldVelocity(0.0f, 300.0f, 0.0f, &dx, &dy);
  Check(FloatNear(dx, 0.0f, 1e-3f) && FloatNear(dy, 300.0f, 1e-3f),
        "yaw=0 时车体前方为世界 +Y");

  AdvanceWorld_BodyToWorldVelocity(0.0f, 300.0f, 90.0f, &dx, &dy);
  Check(FloatNear(dx, -300.0f, 1e-3f) && FloatNear(dy, 0.0f, 1e-3f),
        "yaw=90 时车体前方为世界 -X");

  AdvanceWorld_BodyToWorldVelocity(0.0f, 300.0f, -90.0f, &dx, &dy);
  Check(FloatNear(dx, 300.0f, 1e-3f) && FloatNear(dy, 0.0f, 1e-3f),
        "yaw=-90 时车体前方为世界 +X");
}

static void TestConfigHotload(void)
{
  AdvanceHolonomic_Config_t config;
  AdvanceHolonomic_Config_t active;
  uint32_t revision = 0U;
  uint32_t active_revision = 0U;
  float profile_acceleration;
  WorldGoalPose2D_t goal;

  AdvanceHolonomic_Init();
  (void)AdvanceControl_ReleaseMode();
  Check(AdvanceHolonomic_GetConfig(&active, &active_revision) == ADVANCE_HOLONOMIC_STATUS_OK,
        "读取初始全向配置成功");
  Check(active_revision == 0U, "初始 active revision = 0");
  config = active;
  config.linear_accel_mm_s2 = 1200.0f;
  config.linear_decel_mm_s2 = 1400.0f;
  config.kp_forward = 1.6f;
  config.forward_scale = 1.4f;
  Check(AdvanceHolonomic_RequestConfig(&config, &revision) == ADVANCE_HOLONOMIC_STATUS_OK,
        "提交 pending 全向配置成功");
  Check(revision == 1U, "pending revision = 1");
  (void)AdvanceHolonomic_GetConfig(&active, &active_revision);
  Check(active_revision == 0U && FloatNear(active.kp_forward, 0.8f, 1e-6f),
        "周期边界前仍保持旧 active 配置");

  AdvanceHolonomic_Update();
  (void)AdvanceHolonomic_GetConfig(&active, &active_revision);
  Check(active_revision == revision && FloatNear(active.kp_forward, 1.6f, 1e-6f),
        "20 ms 周期边界切换 active 配置");

  g_test_tick = 2000U;
  SetupFreshPose(0.0f, 0.0f, 0.0f);
  goal = MakeForwardGoal(0.0f, 600.0f, 0.0f);
  Check(AdvanceHolonomic_Start(&goal, 30U) == ADVANCE_HOLONOMIC_STATUS_OK,
        "热加载配置后 Start 成功");
  profile_acceleration = g_holonomic.linear_profile.acceleration;
  config.linear_accel_mm_s2 = 2400.0f;
  config.linear_decel_mm_s2 = 2600.0f;
  config.kp_forward = 2.0f;
  config.forward_scale = 1.8f;
  Check(AdvanceHolonomic_RequestConfig(&config, &revision) == ADVANCE_HOLONOMIC_STATUS_OK,
        "运行中提交第二组配置成功");
  g_test_tick += 20U;
  SetupFreshPose(0.0f, 0.0f, 0.0f);
  AdvanceHolonomic_Update();
  Check(FloatNear(g_holonomic.linear_profile.acceleration, profile_acceleration, 1e-6f),
        "运行中修改 accel 不重建当前轮廓");
  (void)AdvanceHolonomic_GetConfig(&active, &active_revision);
  Check(active_revision == revision && FloatNear(active.kp_forward, 2.0f, 1e-6f),
        "运行中修改 Kp 在下一周期生效");
  AdvanceHolonomic_Cancel();
}

static void TestScaleFinalLimit(void)
{
  AdvanceHolonomic_Reference_t reference = {0};
  AdvanceHolonomic_BodyVelocity_t measured = {0};
  AdvanceHolonomic_BodyVelocity_t command = {0};
  WorldPose2D_t actual = {0};
  AdvanceHolonomic_Config_t config;
  uint32_t revision;
  float norm;

  AdvanceHolonomic_Init();
  (void)AdvanceHolonomic_GetConfig(&config, &revision);
  config.kp_forward = 20.0f;
  config.kp_lateral = 20.0f;
  config.kp_yaw = 20.0f;
  config.forward_scale = 2.0f;
  config.lateral_scale = 2.0f;
  config.yaw_scale = 2.0f;
  (void)AdvanceHolonomic_RequestConfig(&config, &revision);
  AdvanceHolonomic_Update();

  g_holonomic.position_required = 1U;
  g_holonomic.yaw_required = 1U;
  g_holonomic.goal.vmax_mm_s = 100.0f;
  g_holonomic.goal.wmax_deg_s = 30.0f;
  reference.x_mm = 100.0f;
  reference.y_mm = 100.0f;
  reference.yaw_deg = 90.0f;
  actual.origin_ready = 1U;
  actual.valid = 1U;
  AdvanceHolonomic_ComputeBodyCommand(&reference, &actual, &measured, &command);
  norm = sqrtf((command.right_mm_s * command.right_mm_s) +
               (command.forward_mm_s * command.forward_mm_s));
  Check(norm <= 100.0f + 1e-4f, "scale 后平移命令仍受最终 vmax 限幅");
  Check(fabsf(command.wz_deg_s) <= 30.0f + 1e-4f,
        "scale 后航向命令仍受最终 wmax 限幅");
}

static void TestControlCancelRoute(void)
{
  WorldGoalPose2D_t goal;

  AdvanceHolonomic_Init();
  (void)AdvanceControl_ReleaseMode();
  g_test_tick = 3000U;
  SetupFreshPose(0.0f, 0.0f, 0.0f);
  goal = MakeForwardGoal(0.0f, 200.0f, 0.0f);
  Check(AdvanceHolonomic_Start(&goal, 30U) == ADVANCE_HOLONOMIC_STATUS_OK,
        "取消路由测试 Start 成功");
  AdvanceControl_CancelActive();
  Check(g_holonomic_state == ADVANCE_HOLONOMIC_STATE_CANCELED,
        "AdvanceControl_CancelActive 路由到全向取消");
  Check(g_test_mode == ADVANCE_CONTROL_NONE,
        "全向取消后释放控制权");
}

int main(void)
{
  printf("=== advance_holonomic_position host test ===\n");

  TestProfileTrapezoid(100.0f, 300.0f, 600.0f, 800.0f, "triangle-100mm");
  TestProfileTrapezoid(500.0f, 300.0f, 600.0f, 800.0f, "trapezoid-500mm");
  TestProfileTrapezoid(1000.0f, 300.0f, 600.0f, 800.0f, "trapezoid-1000mm");
  TestProfileNegative();
  TestProfileZeroDistance();
  TestProfileYaw();
  TestDtZeroAndSnapshotClear();
  TestBodyToWorldForward();
  TestConfigHotload();
  TestScaleFinalLimit();
  TestControlCancelRoute();

  if (g_failures == 0)
  {
    printf("=== ALL TESTS PASSED ===\n");
    return 0;
  }
  printf("=== %d TEST(S) FAILED ===\n", g_failures);
  return 1;
}

#endif /* ADVANCE_HOLONOMIC_UNIT_TEST */
