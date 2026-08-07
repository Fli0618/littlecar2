#!/usr/bin/env python3
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "pid_tuner"))

from pid_tuner.models import MotionGoal, PidConfigSnapshot
from pid_tuner.serial_client import SerialClient

PORT = "COM5"
BAUD = 115200

# 测试 Y 轴（直行）
TARGET_X = 0.0
TARGET_Y = 1000.0
TARGET_YAW = 0.0

ORIGIN_X = 0.0
ORIGIN_Y = 0.0
ORIGIN_YAW = 0.0

def run_goto_and_record(client: SerialClient, goal: MotionGoal, monitor_duration_s: float = 12.0) -> tuple[float, float, float]:
    telemetry_list = []
    def on_telemetry(t):
        telemetry_list.append(t)
    client.add_telemetry_callback(on_telemetry)
    
    client.goto(goal)
    start_time = time.monotonic()
    
    max_y = -99999.0
    min_y_after_reach = 99999.0
    max_y_after_reach = -99999.0
    
    reached = False
    
    while time.monotonic() - start_time < monitor_duration_s:
        time.sleep(0.05)
        if telemetry_list:
            latest = telemetry_list[-1]
            actual_x, actual_y, actual_yaw = latest.actual
            if actual_y > max_y: max_y = actual_y
            
            dist_to_goal = math.hypot(actual_x - goal.x_mm, actual_y - goal.y_mm)
            vx, vy, wz = latest.measured_velocity
            speed = math.hypot(vx, vy)
            
            if not reached and dist_to_goal < 50.0 and speed < 50.0:
                reached = True
            
            if reached:
                if actual_y < min_y_after_reach: min_y_after_reach = actual_y
                if actual_y > max_y_after_reach: max_y_after_reach = actual_y
                
    client._callbacks.remove(on_telemetry)
    
    overshoot = max(0.0, max_y - goal.y_mm)
    oscillation = (max_y_after_reach - min_y_after_reach) if reached else 0.0
    return max_y, overshoot, oscillation

def execute_test_run(client: SerialClient, round_num: int, kp: float, ki: float, kd: float, base_config: PidConfigSnapshot):
    print(f"\n[{round_num}/10] Testing PID (Straight/Y-Axis): Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
    
    # Apply PID
    new_pid = PidConfigSnapshot(
        kp_pos=kp, ki_pos=ki, kd_pos=kd,
        kp_yaw=base_config.kp_yaw, ki_yaw=base_config.ki_yaw, kd_yaw=base_config.kd_yaw
    )
    client.set_pid(new_pid)
    time.sleep(0.2)
    
    print("  Resetting origin...")
    client.reset_origin()
    time.sleep(0.5)
    
    # Go straight 1000mm
    goal_fwd = MotionGoal(TARGET_X, TARGET_Y, TARGET_YAW, vmax_mm_s=800, wmax_deg_s=90, timeout_ms=8000)
    print(f"  Forward to Y={TARGET_Y}...")
    max_y, overshoot, oscillation = run_goto_and_record(client, goal_fwd, monitor_duration_s=12.0)
    print(f"  --> Max Y: {max_y:.1f}, Overshoot: {overshoot:.1f}mm, Oscillation span: {oscillation:.1f}mm")
    
    client.stop()
    time.sleep(1.0)
    
    # Return to 0
    print("  Returning to origin...")
    goal_ret = MotionGoal(ORIGIN_X, ORIGIN_Y, ORIGIN_YAW, vmax_mm_s=800, wmax_deg_s=90, timeout_ms=8000)
    client.goto(goal_ret)
    time.sleep(6.0)
    client.stop()
    time.sleep(0.5)
    
    return {
        "kp": kp, "ki": ki, "kd": kd,
        "max_y": max_y,
        "overshoot": overshoot,
        "oscillation": oscillation
    }

def main():
    print(f"Connecting to {PORT}...")
    try:
        with SerialClient.open_port(PORT, BAUD, request_timeout_s=0.5) as client:
            print("Connected.")
            current_pid = client.get_pid().config
            
            # Since Y (straight) might have more inertia, we center the tests around
            # the previous best (0.56, 0.01, 0.49) but also test some stronger dampings.
            candidates = [
                (0.80, 0.03, 0.14), # 1. 原版参数对比
                (0.56, 0.01, 0.49), # 2. 横移测试出的最优
                (0.60, 0.01, 0.55), # 3. 略微增强 Kp 和 Kd
                (0.65, 0.01, 0.60), # 4. 增强阻尼，防止直行巨大动能
                (0.70, 0.02, 0.65), # 5. 较高 Kp 保证能走到位，很高 Kd 强力刹车
                (0.75, 0.02, 0.70), # 6. 
                (0.56, 0.00, 0.55), # 7. 去掉积分
                (0.65, 0.00, 0.65), # 8. 去掉积分，高增益
                (0.70, 0.00, 0.70), # 9.
                (0.80, 0.00, 0.80), # 10. 高Kp高Kd对比
            ]
            
            results = []
            for i, (kp, ki, kd) in enumerate(candidates, 1):
                res = execute_test_run(client, i, kp, ki, kd, current_pid)
                results.append(res)
                
            print("\n=== Straight Test Results ===")
            print(f"{'Kp':<6} | {'Ki':<6} | {'Kd':<6} | {'Max Y':<8} | {'Overshoot':<10} | {'Oscillation'}")
            print("-" * 55)
            for r in results:
                print(f"{r['kp']:<6.2f} | {r['ki']:<6.2f} | {r['kd']:<6.2f} | {r['max_y']:<8.1f} | {r['overshoot']:<10.1f} | {r['oscillation']:.1f}")
                
            # Filter ones that reasonably reached the target (Max Y > 990)
            valid_results = [r for r in results if r['max_y'] > 980.0]
            if valid_results:
                best = min(valid_results, key=lambda x: x['overshoot'] + x['oscillation'])
                print(f"\nBest combined performance (reached target): Kp={best['kp']:.2f}, Ki={best['ki']:.2f}, Kd={best['kd']:.2f}")
            else:
                print("\nNo parameter set properly reached the target.")
                
            # 自动把表现最好的参数固化进去
            if valid_results:
                print("\nApplying the best PID parameters...")
                best_pid = PidConfigSnapshot(
                    kp_pos=best['kp'], ki_pos=best['ki'], kd_pos=best['kd'],
                    kp_yaw=current_pid.kp_yaw, ki_yaw=current_pid.ki_yaw, kd_yaw=current_pid.kd_yaw
                )
                client.set_pid(best_pid)
                print("Done!")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
