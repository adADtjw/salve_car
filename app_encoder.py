# ============================================================================
# 闭环电机控制器 — 包含 Clamping 抗饱和 PID、斜坡规划及多轮极性同步校正
# ============================================================================

from smartcar import encoder
from machine import Pin, PWM

class PositionalPID:
    def __init__(self, kp, ki, kd, out_max=100.0):
        """
        位置式 PID 控制器 (采用 Clamping 动态钳位抗饱和算法)
        out_max 默认为 100.0，因为最终输出直接是 0-100 的占空比百分比
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.integral = 0.0
        self.last_err = 0.0

    def calc(self, err):
        # 1. 计算微分项
        derivative = err - self.last_err
        self.last_err = err

        # 2. 计算临时积分与输出
        temp_integral = self.integral + err
        p_term = self.kp * err
        i_term = self.ki * temp_integral
        d_term = self.kd * derivative
        total_out = p_term + i_term + d_term

        # 3. Clamping 动态抗饱和
        if abs(total_out) >= self.out_max and (total_out * err > 0):
            pass # 饱和且误差继续增大时，停止积分
        else:
            self.integral = temp_integral

        # 4. 绝对积分限幅
        if self.ki != 0:
            i_limit = (self.out_max * 0.3) / self.ki
            self.integral = max(min(self.integral, i_limit), -i_limit)

        final_out = p_term + (self.ki * self.integral) + d_term
        return max(min(final_out, self.out_max), -self.out_max)

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0

class DRV8871Motor:
    def __init__(self, in1_pin_name, in2_pin_name, freq=13000, invert=False):
        self.in1 = Pin(in1_pin_name, Pin.OUT, value=0)
        self.in2 = PWM(in2_pin_name, freq, duty_u16=0)
        self.invert = invert

    def _write_forward(self, speed):
        self.in1.value(1)
        duty_ratio = (100.0 - speed) / 100.0
        duty_u16 = int(65535 * duty_ratio)
        self.in2.duty_u16(max(duty_u16, 1))

    def _write_backward(self, speed):
        self.in1.value(0)
        # 闭环中绝对禁止使用 sqrt 等非线性扭曲，纯线性交给 PID 去克服死区
        duty_ratio = speed / 100.0
        duty_u16 = int(65535 * duty_ratio)
        self.in2.duty_u16(max(duty_u16, 1))

    def forward(self, speed):
        if self.invert:
            self._write_backward(speed)
        else:
            self._write_forward(speed)

    def backward(self, speed):
        if self.invert:
            self._write_forward(speed)
        else:
            self._write_backward(speed)

    def brake(self):
        self.in1.value(1)
        self.in2.duty_u16(65535)

class MotorNode:
    # 针对 5700 极速重新调整的参数：
    # Kp=0.03 (1000脉冲误差产生30%驱动力)
    # Ki=0.002 (积分克服反转死区)
    # ramp_limit=200.0 (约 300ms 加速到极速 5700，防止瞬间电流过大拉跨电源)
    def __init__(self, in1, in2, enc_pin1, enc_pin2, invert=False, kp=0.03, ki=0.002, kd=0.01, ramp_limit=200.0):
        self.motor = DRV8871Motor(in1, in2, invert=invert)
        self.enc = encoder(enc_pin1, enc_pin2, invert)
        self.pid = PositionalPID(kp, ki, kd, out_max=100.0)
        
        self.target_speed = 0.0      
        self.current_speed = 0.0     
        self.actual_target = 0.0     
        self.ramp_limit = ramp_limit

    def update(self):
        self.current_speed = self.enc.get()

        if self.target_speed == 0.0 and abs(self.current_speed) < 5.0:
            self.brake()
            return

        # 斜坡规划
        diff = self.target_speed - self.actual_target
        if abs(diff) <= self.ramp_limit:
            self.actual_target = self.target_speed
        else:
            self.actual_target += self.ramp_limit if diff > 0 else -self.ramp_limit

        # PID 计算，输出 0-100 的实际物理占空比
        error = self.actual_target - self.current_speed
        power = self.pid.calc(error)

        if power > 0:
            self.motor.forward(power)
        elif power < 0:
            self.motor.backward(abs(power))
        else:
            self.motor.brake()

    def brake(self):
        self.target_speed = 0.0
        self.actual_target = 0.0
        self.pid.reset()
        self.motor.brake()

class MotorController:
    def __init__(self):
        self.left_node = MotorNode("C28", "C29", "C0", "C1", invert=True)
        self.right_node = MotorNode("C30", "C31", "C2", "C3", invert=False)
        self.head_node = MotorNode("D6", "D7", "D15", "D16", invert=False)

        # 核心映射基准：100% 占空比对应的物理脉冲数极速
        self.MAX_SPEED_PULSE = 4000.0 

    def set_speeds(self, left_duty, right_duty, head_duty):
        """
        上层接口：接收 -100.0 到 100.0 的占空比意图
        """
        # 限制输入范围在合法百分比内
        left_duty = max(min(float(left_duty), 100.0), -100.0)
        right_duty = max(min(float(right_duty), 100.0), -100.0)
        head_duty = max(min(float(head_duty), 100.0), -100.0)

        # 桥接转换：将百分比意图映射为实际脉冲数目标
        self.left_node.target_speed = (left_duty / 100.0) * self.MAX_SPEED_PULSE
        self.right_node.target_speed = (right_duty / 100.0) * self.MAX_SPEED_PULSE
        self.head_node.target_speed = (head_duty / 100.0) * self.MAX_SPEED_PULSE

    def tick(self):
        self.left_node.update()
        self.right_node.update()
        self.head_node.update()

    def brake_all(self):
        self.left_node.brake()
        self.right_node.brake()
        self.head_node.brake()