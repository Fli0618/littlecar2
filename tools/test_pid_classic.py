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

# We will test X=1000, which usually represents forward. 
# If Y is forward, change this to 0 and TARGET_Y = 1000
TARGET_X = 1000.0
TARGET_Y = 0.0
TARGET_YAW = 0.0

ORIGIN_X = 0.0
ORIGIN_Y = 0.0
ORIGIN_YAW = 0.0

def run_goto_and_record(client: SerialClient, goal: MotionGoal, monitor_duration_s: float = 12.0) -> tuple[float, float, float]:
    """Sends goto command and records overshoot and oscillation."""
    telemetry_list = []
    def on_telemetry(t):
        telemetry_list.append(t)
    client.add_telemetry_callback(on_telemetry)
    
    client.goto(goal)
    start_time = time.monotonic()
    
    max_x = -99999.0
    min_x_after_reach = 99999.0
    max_x_after_reach = -99999.0
    
    reached = False
    
    while time.monotonic() - start_time < monitor_duration_s:
        time.sleep(0.05)
        if telemetry_list:
            latest = telemetry_list[-1]
            actual_x, actual_y, actual_yaw = latest.actual
            if actual_x > max_x: max_x = actual_x
            
            dist_to_goal = math.hypot(actual_x - goal.x_mm, actual_y - goal.y_mm)
            vx, vy, wz = latest.measured_velocity
            speed = math.hypot(vx, vy)
            
            if not reached and dist_to_goal < 50.0 and speed < 50.0:
                reached = True
            
            if reached:
                if actual_x < min_x_after_reach: min_x_after_reach = actual_x
                if actual_x > max_x_after_reach: max_x_after_reach = actual_x
                
    client._callbacks.remove(on_telemetry)
    
    overshoot = max(0.0, max_x - goal.x_mm)
    oscillation = (max_x_after_reach - min_x_after_reach) if reached else 0.0
    return max_x, overshoot, oscillation

def execute_test_run(client: SerialClient, round_num: int, kp: float, ki: float, kd: float, base_config: PidConfigSnapshot):
    print(f"\n[{round_num}/10] Testing PID: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
    
    # 1. Apply PID
    new_pid = PidConfigSnapshot(
        kp_pos=kp, ki_pos=ki, kd_pos=kd,
        kp_yaw=base_config.kp_yaw, ki_yaw=base_config.ki_yaw, kd_yaw=base_config.kd_yaw
    )
    client.set_pid(new_pid)
    time.sleep(0.2)
    
    # 2. Reset origin
    print("  Resetting origin...")
    client.reset_origin()
    time.sleep(0.5)
    
    # 3. Go forward 1000mm
    goal_fwd = MotionGoal(TARGET_X, TARGET_Y, TARGET_YAW, vmax_mm_s=800, wmax_deg_s=90, timeout_ms=8000)
    print(f"  Forward to X={TARGET_X}...")
    max_x, overshoot, oscillation = run_goto_and_record(client, goal_fwd, monitor_duration_s=12.0)
    print(f"  --> Max X: {max_x:.1f}, Overshoot: {overshoot:.1f}mm, Oscillation span: {oscillation:.1f}mm")
    
    client.stop()
    time.sleep(1.0)
    
    # 4. Return to 0
    print("  Returning to origin...")
    goal_ret = MotionGoal(ORIGIN_X, ORIGIN_Y, ORIGIN_YAW, vmax_mm_s=800, wmax_deg_s=90, timeout_ms=8000)
    client.goto(goal_ret)
    time.sleep(6.0)
    client.stop()
    time.sleep(0.5)
    
    return {
        "kp": kp, "ki": ki, "kd": kd,
        "overshoot": overshoot,
        "oscillation": oscillation
    }

def main():
    print(f"Connecting to {PORT}...")
    try:
        with SerialClient.open_port(PORT, BAUD, request_timeout_s=0.5) as client:
            print("Connected.")
            current_pid = client.get_pid().config
            print(f"Base PID: {current_pid}")
            
            # Base parameters to explore around
            base_kp = current_pid.kp_pos
            base_kd = current_pid.kd_pos
            base_ki = current_pid.ki_pos
            
            # We want to reduce overshoot and oscillation. 
            # Usually: increase Kd (damping), decrease Kp slightly, or decrease Ki if windup.
            candidates = [
                (base_kp, base_ki, base_kd), # Baseline
                (base_kp * 0.9, base_ki, base_kd * 1.2),
                (base_kp * 0.8, base_ki, base_kd * 1.5),
                (base_kp * 0.8, 0.0, base_kd * 1.5), # Test without Ki
                (base_kp * 0.7, 0.0, base_kd * 2.0),
                (base_kp * 0.6, 0.0, base_kd * 2.5),
                (base_kp * 1.0, base_ki * 0.5, base_kd * 2.0),
                (base_kp * 0.9, base_ki * 0.5, base_kd * 3.0),
                (base_kp * 0.7, base_ki * 0.2, base_kd * 3.5),
                (base_kp * 0.6, base_ki * 0.1, base_kd * 4.0),
            ]
            
            results = []
            for i, (kp, ki, kd) in enumerate(candidates, 1):
                res = execute_test_run(client, i, kp, ki, kd, current_pid)
                results.append(res)
                
            print("\n=== Test Results ===")
            print(f"{'Kp':<6} | {'Ki':<6} | {'Kd':<6} | {'Overshoot':<10} | {'Oscillation'}")
            print("-" * 45)
            for r in results:
                print(f"{r['kp']:<6.2f} | {r['ki']:<6.2f} | {r['kd']:<6.2f} | {r['overshoot']:<10.1f} | {r['oscillation']:.1f}")
                
            best = min(results, key=lambda x: x['overshoot'] + x['oscillation'])
            print(f"\nBest combined performance: Kp={best['kp']:.2f}, Ki={best['ki']:.2f}, Kd={best['kd']:.2f}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
