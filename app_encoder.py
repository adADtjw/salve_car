# ============================================================================
# 闭环电机控制器 — 包含 滤波、Clamping 抗饱和 PID、斜坡规划及极性同步校正
# ============================================================================

from smartcar import encoder
from machine import Pin, PWM

class PositionalPID:
    def __init__(self, kp, ki, kd, out_max=100.0):
        """
        位置式 PID 控制器 (采用 Clamping 动态钳位抗饱和算法 + 测量值微分)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.integral = 0.0
        
        # 优化：弃用 last_err，改用 last_speed，消除目标突变带来的微分突刺
        self.last_speed = 0.0 

    def calc(self, current_speed, target_speed):
        err = target_speed - current_speed

        # 1. 优化微分项：基于测量值的微分 (Derivative on Measurement)
        # 这可以避免斜坡规划或目标速度突变时产生的巨大 D 项突刺，对缓解电机抖动有奇效
        derivative = -(current_speed - self.last_speed)
        self.last_speed = current_speed

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

        # 4. 绝对积分限幅 (防止卡死时积分无限制增加)
        if self.ki != 0:
            i_limit = (self.out_max * 0.4) / self.ki
            self.integral = max(min(self.integral, i_limit), -i_limit)

        final_out = p_term + (self.ki * self.integral) + d_term
        return max(min(final_out, self.out_max), -self.out_max)

    def reset(self):
        self.integral = 0.0
        self.last_speed = 0.0


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
    def __init__(self, in1, in2, enc_pin1, enc_pin2, invert=False, kp=0.03, ki=0.002, kd=0.01, ramp_limit=200.0):
        self.motor = DRV8871Motor(in1, in2, invert=invert)
        self.enc = encoder(enc_pin1, enc_pin2, invert)
        self.pid = PositionalPID(kp, ki, kd, out_max=100.0)
        
        self.target_speed = 0.0      
        self.current_speed = 0.0     
        self.actual_target = 0.0     
        self.ramp_limit = ramp_limit

    def update(self):
        raw_speed = self.enc.get()
        alpha = 0.35 
        self.current_speed = (1.0 - alpha) * self.current_speed + alpha * raw_speed

        if self.target_speed == 0.0 and abs(self.current_speed) < 5.0:
            self.brake()
            return

        # 斜坡规划
        diff = self.target_speed - self.actual_target
        if abs(diff) <= self.ramp_limit:
            self.actual_target = self.target_speed
        else:
            self.actual_target += self.ramp_limit if diff > 0 else -self.ramp_limit

        # ==================== 新增：前馈控制 (Feedforward) ====================
        # 假设最高速度是 4000 脉冲，对应 100% 占空比，那么前馈系数 Kf 就是 100 / 4000 = 0.025
        Kf = 0.025
        feedforward_power = self.actual_target * Kf
        
        # PID 现在只需要负责纠正“前馈猜不准”的那一点点误差
        pid_power = self.pid.calc(self.current_speed, self.actual_target)
        
        # 最终输出 = 前馈基础出力 + PID 微调
        power = feedforward_power + pid_power
        # ======================================================================

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