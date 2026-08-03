#!/usr/bin/env python3
"""Jetson Orin Nano 物理脚 32（GPIO07）补光灯 PWM 测试工具。

接线：灯的控制输入接物理脚 32，灯和 Jetson 共地（物理脚 34/39 等）。
如果灯的工作电流超过 GPIO 能提供的电流，请使用三极管/MOSFET 驱动，
不要直接从 GPIO 给灯供电。

直接给100%就好了
"""

import argparse
import sys
import time

import Jetson.GPIO as GPIO


PWM_PIN = 32          # BOARD 编号：40 针排针的物理脚 32，也就是 GPIO07
PWM_FREQUENCY_HZ = 1000
DEFAULT_STEP = 5
DEFAULT_HOLD_SECONDS = 2.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="遍历补光灯 PWM 占空比")
    parser.add_argument(
        "--step",
        type=float,
        default=DEFAULT_STEP,
        help="占空比步进，范围 0~100，默认 5%%",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=DEFAULT_HOLD_SECONDS,
        help="每档亮度持续秒数，默认 2 秒",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=PWM_FREQUENCY_HZ,
        help="PWM 频率，默认 1000 Hz",
    )
    return parser.parse_args()


def duty_values(step: float) -> list[float]:
    """生成包含 0% 和 100% 的占空比列表，避免浮点步进遗漏终点。"""
    values = []
    value = 0.0
    while value < 100.0:
        values.append(round(value, 2))
        value += step
    values.append(100.0)
    return values


def main() -> int:
    args = parse_args()
    if not 0 < args.step <= 100:
        raise ValueError("--step 必须大于 0 且不超过 100")
    if args.hold < 0:
        raise ValueError("--hold 不能小于 0")
    if args.frequency <= 0:
        raise ValueError("--frequency 必须大于 0")

    pwm = None
    selected_duty = None
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
        pwm = GPIO.PWM(PWM_PIN, args.frequency)
        pwm.start(0)

        print(f"PWM 输出：物理脚 {PWM_PIN}（GPIO07），频率 {args.frequency:g} Hz")
        print("每档亮度会保持指定时间；按回车进入下一档，输入 q 立即退出。")
        print("注意：LED 亮度与占空比通常不是线性关系，选出合适档位即可。\n")

        for duty in duty_values(args.step):
            pwm.ChangeDutyCycle(duty)
            print(f"当前占空比：{duty:6.2f}%")
            if args.hold:
                time.sleep(args.hold)
            command = input("回车继续，输入 q 退出，或输入占空比保存并退出：").strip()
            if command.lower() == "q":
                break
            if command:
                selected_duty = float(command)
                if not 0 <= selected_duty <= 100:
                    raise ValueError("占空比必须在 0~100 之间")
                pwm.ChangeDutyCycle(selected_duty)
                print(f"已设置选定占空比：{selected_duty:.2f}%")
                input("观察确认后按回车结束测试……")
                break
        return 0
    except KeyboardInterrupt:
        print("\n用户中断测试。")
        return 130
    finally:
        if pwm is not None:
            pwm.ChangeDutyCycle(0)
            pwm.stop()
        GPIO.cleanup()
        print("PWM 已关闭，GPIO 已清理。")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f"运行失败：{exc}", file=sys.stderr)
        raise SystemExit(1)