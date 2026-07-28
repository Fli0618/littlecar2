"""Jetson 补光灯 PWM 点灯测试。"""

from __future__ import annotations

import time

import Jetson.GPIO as GPIO


# 用户直接修改本区常量即可调整补光灯亮度。
PWM_PIN = 33
PWM_FREQUENCY_HZ = 10000
PWM_DUTY_CYCLE_PERCENT = 50.0


def main() -> None:
    if PWM_FREQUENCY_HZ <= 0:
        raise ValueError("PWM_FREQUENCY_HZ 必须大于 0")
    if not 0.0 <= PWM_DUTY_CYCLE_PERCENT <= 100.0:
        raise ValueError("PWM_DUTY_CYCLE_PERCENT 必须在 0 到 100 之间")

    pwm = None
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
        # 补光灯独立供电时，控制地必须与 Jetson GND 共地。
        pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY_HZ)
        pwm.start(PWM_DUTY_CYCLE_PERCENT)
        print(
            f"补光灯已开启: Pin {PWM_PIN}, "
            f"频率 {PWM_FREQUENCY_HZ} Hz, 占空比 {PWM_DUTY_CYCLE_PERCENT:.1f}%"
        )
        print("按 Ctrl+C 退出。")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n收到退出信号，正在关闭补光灯。")
    except Exception as exc:
        if pwm is None:
            print(
                "PWM 初始化失败，请检查 Pin 33 是否已通过 Jetson-IO 配置为 PWM，"
                "以及当前用户是否具有 GPIO 权限。"
            )
        raise RuntimeError(f"补光灯测试失败: {exc}") from exc
    finally:
        if pwm is not None:
            try:
                pwm.ChangeDutyCycle(0)
                pwm.stop()
            except Exception as exc:
                print(f"停止 PWM 时发生错误: {exc}")
        try:
            GPIO.output(PWM_PIN, GPIO.LOW)
        finally:
            GPIO.cleanup(PWM_PIN)


if __name__ == "__main__":
    main()
