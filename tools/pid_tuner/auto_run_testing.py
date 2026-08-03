#!/usr/bin/env python3
"""Auto-tuning script with back-and-forth motion, returning to origin each time."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from queue import Empty

from pid_tuner.models import MotionGoal, PidConfig
from pid_tuner.serial_client import SerialClient
from pid_tuner.storage import DEFAULT_LOGS_DIR, write_telemetry_csv

# 安全运动边界
SAFE_VMAX = 450.0  # mm/s
SAFE_WMAX = 50.0   # deg/s
MOTION_Y_MM = 2000.0  # 目标单向距离


def run_cycle(client: SerialClient, port: str, y_target: float, csv_path: Path | None = None) -> list:
    """运行单次到目标点的闭环运动，并收集高频遥测（Telemetry）数据。"""
    goal = MotionGoal(
        x_mm=0.0,
        y_mm=y_target,
        yaw_deg=0.0,
        vmax_mm_s=SAFE_VMAX,
        wmax_deg_s=SAFE_WMAX,
        timeout_ms=8000,
        use_yaw=True,
    )

    telemetry = []
    print(f"--> 下发目标: Y = {y_target} mm, Vmax = {SAFE_VMAX} mm/s")
    client.goto(goal)

    deadline = time.monotonic() + 9.0  # 8000ms 超时+1秒缓冲
    heartbeat_at = time.monotonic() + 0.5
    last_telemetry = time.monotonic()

    # 循环收集波形帧并维持心跳
    while time.monotonic() < deadline:
        try:
            item = client.get_telemetry(timeout_s=0.05)
            telemetry.append(item)
            last_telemetry = time.monotonic()

            # 定时在后台终端简洁地流式输出状态
            if len(telemetry) % 15 == 0:
                print(f"  [运行中] 秒数={item.tick/1000.0:.2f}s 状态={item.state} 实际Y={item.actual[1]:.1f}mm 误差=[{item.error[0]:.1f}, {item.error[1]:.1f}]mm", flush=True)

            # 状态2表示下位机已判定“已到达并停稳目标点”
            if item.state == 2:
                print(f"  [通知] 下位机报告已成功到达 Y = {y_target} 附近！", flush=True)
                break
        except Empty:
            pass

        # 维持链路心跳防止断开
        if time.monotonic() >= heartbeat_at:
            client.heartbeat()
            heartbeat_at = time.monotonic() + 0.5

        # 超时容错
        if telemetry and (time.monotonic() - last_telemetry) >= 1.5:
            print("  [警告] 丢失下位机遥测心跳！")
            break

    # 停止动作
    try:
        client.stop()
    except Exception:
        pass

    # 物理静止稳定时间
    time.sleep(1.0)

    # 导出单轮 CSV 位姿历史
    if csv_path and telemetry:
        write_telemetry_csv(csv_path, telemetry)
        print(f"  [存储] 遥测波形已写入到 {csv_path.name}")

    return telemetry


def main() -> int:
    parser = argparse.ArgumentParser(description="自动往返控制调试测试器")
    parser.add_argument("--port", default="COM5", help="串口号，例如 COM5")
    parser.add_argument("--cycles", type=int, default=3, help="运行的往返大轮数数量")
    args = parser.parse_args()

    print("==============================================")
    print("      🚂 麦克纳姆底盘自动化往返与零点回归调试器 🚂")
    print("==============================================")
    print(f"工作模式：每次跑完 {MOTION_Y_MM} mm 后，都先【重归物理解算原点 (0,0)】，再跑下一轮。")
    print(f"通信接口: {args.port}  轮数设定: {args.cycles}")

    try:
        # 实例化后台串口客户端
        client = SerialClient.open_port(args.port, baud=115200)
    except Exception as e:
        print(f"❌ 无法打开串口 {args.port}: {e}", file=sys.stderr)
        return 1

    with client:
        # 1. 重置初始软件原点
        print("\n[正在归零] 重置初始自由轮（OPS）软件坐标系原点...")
        client.reset_origin()
        time.sleep(1.5)  # 摆正稳定

        # 获取当前的 PID 参数版本
        try:
            rev, pid = client.get_pid()
            print(f"🔑 当前活动 PID [版本 {rev}]: \n  位置 => Kp={pid.kp_pos:.2f}, Ki={pid.ki_pos:.2f}, Kd={pid.kd_pos:.2f}\n  航向 => Kp={pid.kp_yaw:.2f}, Ki={pid.ki_yaw:.2f}, Kd={pid.kd_yaw:.2f}\n")
        except Exception as e:
            print(f"⚠️ 读取当前 PID 参数出现小超时，直接开始执行周期动作。")

        for iteration in range(1, args.cycles + 1):
            print(f"\n⚡ ===== 【第 {iteration} 轮测试开始】 =====")

            # --- A. 出发前往目标点 2000mm ---
            csv_forward = DEFAULT_LOGS_DIR / f"cycle_{iteration}_forward.csv"
            fwd_tel = run_cycle(client, args.port, MOTION_Y_MM, csv_forward)

            if fwd_tel:
                last_node = fwd_tel[-1]
                print(f"📍 出发终点实测：Y到达位置={last_node.actual[1]:.2f}mm (偏差={last_node.error[1]:.2f}mm)")
                print(f"📍 侧偏：X绝对偏差={abs(last_node.actual[0]):.2f}mm  角度偏航={last_node.actual[2]:.2f}度")

            print("\n⏰ 准备返回原点中...")
            time.sleep(1.0)

            # --- B. 强制返回 0 点 确保完全回归初始点 0  ---
            csv_return = DEFAULT_LOGS_DIR / f"cycle_{iteration}_back_to_zero.csv"
            back_tel = run_cycle(client, args.port, 0.0, csv_return)

            if back_tel:
                last_node = back_tel[-1]
                print(f"🏁 零点回归完毕：最终残留位置Y={last_node.actual[1]:.2f}mm (残余偏差={last_node.error[1]:.2f}mm)")
                print(f"🏁 残留侧向偏位：X={last_node.actual[0]:.2f}mm  残余偏角={last_node.actual[2]:.2f}度")

            print(f"✨ ===== 【第 {iteration} 轮测试已安全闭环完成】 =====\n")
            time.sleep(2.0)  # 轮间充分静止

    print("📢 自动化往返闭环调度测试已成功执行完成！")
    return 0


if __name__ == "__main__":
    sys.exit(main())
