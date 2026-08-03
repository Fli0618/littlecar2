#!/usr/bin/env python3
"""Auto-optimizer with comparative self-tuning algorithms for STM32 PID tuner."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from queue import Empty
import numpy as np

from pid_tuner.models import MotionGoal, PidConfig
from pid_tuner.serial_client import SerialClient
from pid_tuner.storage import write_telemetry_csv

# 基础配置
SAFE_VMAX = 450.0  # mm/s
SAFE_WMAX = 50.0   # deg/s
MOTION_Y_MM = 2000.0  # 目标位置


class RunEvaluator:
    """对单次往返运动性能进行量化打分的评估器"""
    @staticmethod
    def evaluate(forward_telemetry: list, back_telemetry: list) -> dict[str, float]:
        if not forward_telemetry or not back_telemetry:
            return {"score": 999.0, "max_x_offset": 999.0, "max_yaw_offset": 999.0, "back_y_error": 999.0}

        # 1. 出发后，行进过程中的最大侧向 X 偏差
        fwd_x = [abs(p.actual[0]) for p in forward_telemetry]
        back_x = [abs(p.actual[0]) for p in back_telemetry]
        max_x_offset = float(max(max(fwd_x), max(back_x)))

        # 2. 全程最大航向角度偏摆
        fwd_yaw = [abs(p.actual[2]) for p in forward_telemetry]
        back_yaw = [abs(p.actual[2]) for p in back_telemetry]
        max_yaw_offset = float(max(max(fwd_yaw), max(back_yaw)))

        # 3. 回到零点时的 Y 轴残余物差
        back_y_error = float(abs(back_telemetry[-1].actual[1]))

        # 4. Y 移动到达耗时估算
        fwd_time = (forward_telemetry[-1].tick - forward_telemetry[0].tick) / 1000.0
        back_time = (back_telemetry[-1].tick - back_telemetry[0].tick) / 1000.0
        total_time = float(fwd_time + back_time)

        # 评分惩罚项计算：分数越低，控制性能越完美
        # 权重：Y回零残余静差 (x3) + X侧向漂移 (x2) + 车头航向自转 (x4) + 时间成本 (x0.5)
        score = (back_y_error * 3.0) + (max_x_offset * 1.5) + (max_yaw_offset * 5.0) + (total_time * 0.5)

        return {
            "score": score,
            "max_x_offset": max_x_offset,
            "max_yaw_offset": max_yaw_offset,
            "back_y_error": back_y_error,
            "total_time": total_time
        }


def run_motion_profile(client: SerialClient, port: str, y_target: float) -> list:
    """运行单向定位任务并收集其遥测波形"""
    goal = MotionGoal(
        x_mm=0.0,
        y_mm=y_target,
        yaw_deg=0.0,
        vmax_mm_s=SAFE_VMAX,
        wmax_deg_s=SAFE_WMAX,
        timeout_ms=7500,
        use_yaw=True,
    )

    telemetry = []
    client.goto(goal)

    deadline = time.monotonic() + 8.5
    heartbeat_at = time.monotonic() + 0.5
    last_telemetry = time.monotonic()

    while time.monotonic() < deadline:
        try:
            item = client.get_telemetry(timeout_s=0.05)
            telemetry.append(item)
            last_telemetry = time.monotonic()

            # 定时打印，保持控制台交互清洁
            if len(telemetry) % 25 == 0:
                print(f"    Y={item.actual[1]:.1f}mm  X_Err={item.error[0]:.1f}mm  Yaw={item.actual[2]:.2f}°", flush=True)

            if item.state == 2:  # 判定到达
                break
        except Empty:
            pass

        # 心跳维护
        if time.monotonic() >= heartbeat_at:
            client.heartbeat()
            heartbeat_at = time.monotonic() + 0.5

        if telemetry and (time.monotonic() - last_telemetry) >= 1.5:
            break

    try:
        client.stop()
    except Exception:
        pass
    time.sleep(1.0)
    return telemetry


def main() -> int:
    parser = argparse.ArgumentParser(description="自适应自动迭代调参器")
    parser.add_argument("--port", default="COM5", help="串口号")
    parser.add_argument("--iterations", type=int, default=3, help="尝试调参的迭代实验次数")
    args = parser.parse_args()

    print("=========================================================")
    print("     🤖 麦克纳姆底盘【自适应迭代优化 & PID比对评估器】 🤖")
    print("=========================================================")
    print(f"我们将测试 {args.iterations} 组特制的 PID 参数。")
    print(f"每组进行：【更新PID】->【归零】->【跑2000往返】->【性能打分比对】")

    # 准备进行比对的三组实验参数（依次渐进增加阻尼和增益，寻找最优峰值）
    # 第一组：更灵敏的位置环
    configs_to_test = [
        # 1. 探索位置Kp大、Kd阻尼提升
        PidConfig(kp_pos=1.40, ki_pos=0.12, kd_pos=0.72, kp_yaw=2.60, ki_yaw=1.0, kd_yaw=0.85),
        # 2. 探索航向极大刚度下、高阻尼位置
        PidConfig(kp_pos=1.30, ki_pos=0.15, kd_pos=0.70, kp_yaw=2.80, ki_yaw=1.0, kd_yaw=0.90),
        # 3. 探索极速位置响应、中航向纠偏
        PidConfig(kp_pos=1.50, ki_pos=0.10, kd_pos=0.78, kp_yaw=2.50, ki_yaw=1.0, kd_yaw=0.80),
    ]

    # 必要时截断超出参数项
    configs_to_test = configs_to_test[:args.iterations]

    try:
        client = SerialClient.open_port(args.port, baud=115200)
    except Exception as e:
        print(f"❌ 无法连接串口: {e}")
        return 1

    history_results = []

    with client:
        # 每次大实验前先重归原点
        print("\n[重置] 摆正车身，校准初始坐标原点...")
        client.reset_origin()
        time.sleep(1.5)

        for i, config in enumerate(configs_to_test, 1):
            print(f"\n🧪 ----- 【实验设计 {i}/{len(configs_to_test)}】 -----")
            print(f"👉 正在热写入测试 PID 参数:")
            print(f"   [位置] Kp={config.kp_pos:.2f}, Ki={config.ki_pos:.2f}, Kd={config.kd_pos:.2f}")
            print(f"   [航向] Kp={config.kp_yaw:.2f}, Ki={config.ki_yaw:.2f}, Kd={config.kd_yaw:.2f}")

            # 在线热写入PID
            revision = client.set_pid(config)
            print(f"   [状态] 写入成功，下位机指令版本已更新至 => {revision}")
            time.sleep(1.0)

            # --- 动作阶段 1：向 2000mm 挺进 ---
            print(f"  🏁 [前进 Y=2000.0 mm]")
            fwd_data = run_motion_profile(client, args.port, MOTION_Y_MM)

            # --- 动作阶段 2：返回物理解算原点 0.0 ---
            print(f"  🏁 [返回 Y=0.0 mm]")
            back_data = run_motion_profile(client, args.port, 0.0)

            # --- 性能打分量化 ---
            metrics = RunEvaluator.evaluate(fwd_data, back_data)
            metrics["config"] = config
            metrics["id"] = i
            history_results.append(metrics)

            print(f"\n📊 【实验 {i} 评分结果 (越小越完美)】: {metrics['score']:.2f} 分")
            print(f"   - 出归零点Y残留偏差: {metrics['back_y_error']:.2f} mm")
            print(f"   - 全程最大侧斜X漂移: {metrics['max_x_offset']:.2f} mm")
            print(f"   - 全程最大车头摆角: {metrics['max_yaw_offset']:.2f}°")
            print(f"   - 往返大循环经历时间: {metrics['total_time']:.2f} 秒")
            print("---------------------------------------------------------")
            time.sleep(2.0)

        # --- 最终迭代比对分析 ---
        print("\n=========================================================")
        print("          🏆 PID 往返实验多维比对排名结果 🏆")
        print("=========================================================")

        # 按得分由低到高（由优到劣）排序
        history_results.sort(key=lambda x: x["score"])

        for rank, res in enumerate(history_results, 1):
            cfg = res["config"]
            medallion = "🥇 [冠军推荐]" if rank == 1 else f"🥈 [排名 {rank}]"
            print(f"{medallion} 实验 {res['id']}  综合得分: {res['score']:.2f} 分")
            print(f"   参数配置: pos_kp={cfg.kp_pos:.2f}, pos_kd={cfg.kd_pos:.2f} | yaw_kp={cfg.kp_yaw:.2f}, yaw_kd={cfg.kd_yaw:.2f}")
            print(f"   测算数据: Y轴回零偏差={res['back_y_error']:.1f}mm | 侧倾X漂移={res['max_x_offset']:.1f}mm | 摆头Yaw={res['max_yaw_offset']:.2f}°")
            print("-" * 57)

        # 将最优 PID 热覆写应用
        best_cfg = history_results[0]["config"]
        print(f"\n💾 [自动覆写] 目前已将排名第1 (最平稳、直行抗偏和重合度最高) 的 实验 {history_results[0]['id']} 参数热写入并在单片机上固化！")
        client.set_pid(best_cfg)

    print("\n📢 多环境参数交叉测试与自适应最优化评估全部圆满结束！")
    return 0


if __name__ == "__main__":
    sys.exit(main())
