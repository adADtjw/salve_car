# ============================================================================
# 电机驱动底层 — GPIO + PWM 控制 (DRV8871)
# DRV8871 真值表:
#   IN1=0, IN2=0 → 滑行/休眠
#   IN1=1, IN2=0 → 正转
#   IN1=0, IN2=1 → 反转
#   IN1=1, IN2=1 → 制动 (慢衰减)
#
# 正转: IN1=1, IN2=PWM → off 态=制动 → 慢衰减 (扭矩大)
# 反转: IN1=0, IN2=PWM → off 态=滑行 → 快衰减 (扭矩小)
#
# 反转快衰减补偿: 扭矩 ∝ D² (D=占空比), 故用 sqrt 映射
#   输入 speed% → 实际 duty = sqrt(speed/100) * 100%
# ============================================================================

from machine import Pin, PWM

# ---- 硬件引脚定义 ----
# IN1: 方向 (GPIO), IN2: 调速 (PWM)
right_in1 = Pin("C30", Pin.OUT, value=0)
right_in2 = PWM("C31", 13000, duty_u16=0)

left_in1 = Pin("C28", Pin.OUT, value=0)
left_in2 = PWM("C29", 13000, duty_u16=0)

head_in1 = Pin("D6", Pin.OUT, value=0)
head_in2 = PWM("D7", 13000, duty_u16=0)


def forward(in1_pin, in2_pwm, speed):
    """
    电机正转 (慢衰减调速)。
    IN1=常高, IN2=PWM((100-speed)%) → off 态为制动
    speed: 0~100
    """
    in1_pin.value(1)
    duty = int(65535 * (100 - speed) / 100)
    in2_pwm.duty_u16(max(duty, 1))


def backward(in1_pin, in2_pwm, speed):
    """
    电机反转 (全速, 无 PWM — 调试版)
    强制 IN1=0, IN2=100% → DRV8871 全速反转
    先确认硬件能否反转, 能反转后再恢复调速
    """
    in1_pin.low()
    in2_pwm.duty_u16(65535)  # 100% duty = full reverse


def brake(in1_pin, in2_pwm):
    """
    电机制动。
    IN1=1, IN2=100% → 短路制动
    """
    in1_pin.value(1)
    in2_pwm.duty_u16(65535)
