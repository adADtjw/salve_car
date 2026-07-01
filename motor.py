# ============================================================================
# 电机驱动底层 — GPIO + PWM 控制
# 每个电机 2 根线: IN1 (方向) + IN2 (PWM 调速/制动)
# ============================================================================

from machine import Pin, PWM

# ---- 硬件引脚定义 ----
# 右电机: C30 (方向) + C31 (PWM)
right_in1 = Pin("C30", Pin.OUT)
right_in2 = PWM("C31", 13000, duty_u16=1)

# 左电机: C28 (方向) + C29 (PWM)
left_in1 = Pin("C28", Pin.OUT)
left_in2 = PWM("C29", 13000, duty_u16=1)

# 尾轮电机: D6 (方向) + D7 (PWM)
head_in1 = Pin("D6", Pin.OUT)
head_in2 = PWM("D7", 13000, duty_u16=1)


def forward(in1_pin, in2_pwm, speed):
    """
    电机正转 (调速)。
    IN1=高电平, IN2 PWM 占空比 = (100 - speed)%
    speed: 0~100 (占空比百分比)
    """
    in1_pin.value(1)
    duty = int(65535 * (100 - speed) / 100)
    in2_pwm.duty_u16(max(duty, 1))


def backward(in1_pin, in2_pwm, speed):
    """
    电机反转 (调速)。
    IN1=低电平, IN2 PWM 占空比 = speed%
    speed: 0~100
    """
    in1_pin.value(0)
    duty = int(65535 * speed / 100)
    in2_pwm.duty_u16(max(duty, 1))


def brake(in1_pin, in2_pwm):
    """
    电机制动 (快速停止)。
    IN1=1, IN2 PWM 占空比=100% → H 桥短路制动
    """
    in1_pin.value(1)
    in2_pwm.duty_u16(65535)
