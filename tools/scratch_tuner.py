import time
import math
import sys
import threading
from collections import deque
from dataclasses import replace

from pid_tuner.serial_client import SerialClient
from pid_tuner.protocol import HolonomicTelemetry, HolonomicConfig
from pid_tuner.models import MotionGoal

def main():
    port = "COM5"
    baud = 115200
    
    print(f"Connecting to {port} at {baud} baud...")
    client = SerialClient.open_port(port, baud)
    
    telemetry_buffer = deque(maxlen=1000)
    current_state = 0
    state_lock = threading.Lock()
    
    def on_telemetry(t: HolonomicTelemetry):
        nonlocal current_state
        telemetry_buffer.append(t)
        with state_lock:
            current_state = t.state
            
    client.add_holonomic_telemetry_callback(on_telemetry)
    client.start()
    
    # Wait for connection
    print("Waiting for connection and telemetry...")
    time.sleep(2)
    
    # Get current config
    config_state = client.get_holonomic_config()
    config = config_state.config
    print(f"Initial Config: {config}")
    
    # Optimization parameters
    best_score = float('inf')
    best_config = None
    
    # Set initial optimal base config
    config = replace(config, linear_decel_mm_s2=800.0, kp_forward=1.15, kv_forward=0.30)
    client.set_holonomic_config(config)
    time.sleep(0.2)
    
    # Target: Fast response, precise and stable stop
    for round_num in range(1, 11):
        print(f"\n--- Round {round_num} ---")
        print(f"Testing Config: decel={config.linear_decel_mm_s2:.1f}, kp={config.kp_forward:.2f}, kv={config.kv_forward:.2f}")
        
        # Go to -1500, 1000
        goal = MotionGoal(
            x_mm=-1500.0, y_mm=1000.0, yaw_deg=0.0,
            vmax_mm_s=1500.0, wmax_deg_s=120.0,
            timeout_ms=5000,
            use_position=True, use_yaw=True
        )
        print("Moving to (-1500, 1000)...")
        telemetry_buffer.clear()
        client.holonomic_goto(goal)
        
        # Wait for movement to finish
        time.sleep(0.5) # Let it start
        while True:
            with state_lock:
                s = current_state
            if s == 2: # ARRIVED
                break
            time.sleep(0.05)
            
        print("Arrived at (-1500, 1000), waiting for settling...")
        time.sleep(1.0) # wait for settling
        client.stop()
        time.sleep(0.1)
        
        # Calculate overshoot & oscillation
        samples = list(telemetry_buffer)
        
        max_x = min(s.actual[0] for s in samples) # X goes negative, so min is the furthest point
        overshoot = -1500.0 - max_x # if max_x is -1600, overshoot is 100
        
        # Calculate oscillation (variance of angular velocity near the end)
        settling_samples = samples[-20:] # Last 20 samples (~0.8s)
        wz_list = [s.measured[2] for s in settling_samples]
        if wz_list:
            wz_mean = sum(wz_list) / len(wz_list)
            wz_var = sum((w - wz_mean)**2 for w in wz_list) / len(wz_list)
        else:
            wz_var = 0
            
        print(f"Overshoot: {overshoot:.1f} mm, Oscillation (wz variance): {wz_var:.2f}")
        
        # Score = absolute overshoot + oscillation penalty
        score = abs(overshoot) + wz_var * 5
        if score < best_score:
            best_score = score
            best_config = HolonomicConfig(**config.__dict__)
            
        # Go back to -500, 1000
        goal_back = MotionGoal(
            x_mm=-500.0, y_mm=1000.0, yaw_deg=0.0,
            vmax_mm_s=1500.0, wmax_deg_s=120.0,
            timeout_ms=5000,
            use_position=True, use_yaw=True
        )
        print("Moving back to (-500, 1000)...")
        telemetry_buffer.clear()
        client.holonomic_goto(goal_back)
        time.sleep(0.5)
        while True:
            with state_lock:
                s = current_state
            if s == 2:
                break
            time.sleep(0.05)
        time.sleep(0.5)
        client.stop()
        time.sleep(0.1)
        
        # Adjust parameters for next round
        # We sweep Kv upwards by 0.05 each round to find the best damping
        config = replace(config, kv_forward=config.kv_forward + 0.05, kv_lateral=config.kv_lateral + 0.05, kv_yaw=config.kv_yaw + 0.05)
            
        client.set_holonomic_config(config)
        time.sleep(0.2)

    print("\n--- Tuning Complete ---")
    print(f"Best Config found (Score: {best_score:.2f}):")
    print(f"Decel: {best_config.linear_decel_mm_s2:.1f} mm/s^2")
    print(f"Kp Forward: {best_config.kp_forward:.2f}")
    print(f"Kv Forward: {best_config.kv_forward:.2f}")
    
    # Apply best config
    client.set_holonomic_config(best_config)
    client.stop()
    client.close()

if __name__ == "__main__":
    main()
