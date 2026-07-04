# ============================================================================
# 闭环电机控制器 — 包含 Clamping 抗饱和 PID、斜坡规划及多轮极性同步校正
# ============================================================================

from smartcar import encoder
from machine import Pin, PWM

class PositionalPID:
    def __init__(self, kp, ki, kd, out_max=300.0):
        """
        位置式 PID 控制器 (采用 Clamping 动态钳位抗饱和算法)
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

        # 2. 计算不考虑 Clamping 限制时的临时积分值和各项输出
        temp_integral = self.integral + err
        p_term = self.kp * err
        i_term = self.ki * temp_integral
        d_term = self.kd * derivative
        total_out = p_term + i_term + d_term

        # 3. 经典的 Clamping 动态抗饱和逻辑
        if abs(total_out) >= self.out_max and (total_out * err > 0):
            pass
        else:
            self.integral = temp_integral

        # 4. 限制积分项本身的最大物理范围
        if self.ki != 0:
            i_limit = (self.out_max * 0.3) / self.ki
            self.integral = max(min(self.integral, i_limit), -i_limit)

        # 5. 计算最终的闭环输出
        final_out = p_term + (self.ki * self.integral) + d_term
        return max(min(final_out, self.out_max), -self.out_max)

    def reset(self):
        """复位内部积分与历史误差"""
        self.integral = 0.0
        self.last_err = 0.0

class DRV8871Motor:
    def __init__(self, in1_pin_name, in2_pin_name, freq=13000, invert=False):
        self.in1 = Pin(in1_pin_name, Pin.OUT, value=0)
        self.in2 = PWM(in2_pin_name, freq, duty_u16=0)
        self.invert = invert

    def _write_forward(self, speed):
        self.in1.value(1)
        duty_ratio = speed / 100.0
        duty_u16 = int(65535 * duty_ratio)
        self.in2.duty_u16(max(duty_u16, 1))

    def _write_backward(self, speed):
        self.in1.value(0)
        duty_ratio = speed / 100.0
        duty_u16 = int(65535 * duty_ratio)
        self.in2.duty_u16(duty_u16)

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
    def __init__(self, in1, in2, enc_pin1, enc_pin2, invert=False, kp=0.2, ki=0.03, kd=0.15, ramp_limit=10.0):
        """
        单路闭环电机节点
        :param in1, in2: DRV8871 控制引脚
        :param enc_pin1, enc_pin2: 编码器 AB 相硬件引脚
        :param invert: 是否镜像取反 (True 则底层电机驱动与编码器读数均自动同步反向)
        :param ramp_limit: 每 10ms 允许的最大速度增量 (斜坡规划防止电流骤降)
        """
        # 初始化电机驱动和编码器，并将极性反转直接交由底层接管
        self.motor = DRV8871Motor(in1, in2, invert=invert)
        self.enc = encoder(enc_pin1, enc_pin2, invert)
        self.pid = PositionalPID(kp, ki, kd, out_max=100.0)
        
        # 暴露给 controller.py 在直接占空比模式 (MANUAL_DUTY) 中使用的属性映射
        self.in1 = self.motor.in1
        self.in2 = self.motor.in2
        self.motor_invert = invert
        self.invert = invert

        self.target_speed = 0.0      # 上层下发的目标测速 (单位：脉冲数/10ms)
        self.current_speed = 0.0     # 校正后的实际测速 (单位：脉冲数/10ms)
        self.actual_target = 0.0     # 经过斜坡过渡后的实际目标值
        self.ramp_limit = ramp_limit

    def update(self):
        """
        核心周期轮询函数 (每 10ms 稳定调用一次)
        """
        # 1. 读取并清空编码器寄存器。
        # 由于初始化时已将 invert 传给底层 encoder，此处读到的 raw_pulses 已经是极性修正后的值
        raw_pulses = self.enc.get()
        self.current_speed = raw_pulses

        # 2. 零速静止快速锁死机制
        if self.target_speed == 0.0 and abs(self.current_speed) < 1.5:
            self.brake()
            return

        # 3. 斜坡曲线规划 (Ramp)
        diff = self.target_speed - self.actual_target
        if abs(diff) <= self.ramp_limit:
            self.actual_target = self.target_speed
        else:
            self.actual_target += self.ramp_limit if diff > 0 else -self.ramp_limit

        # 4. 闭环 PID 计算
        error = self.actual_target - self.current_speed
        power = self.pid.calc(error)

        # 5. 直接通过开环 1:1 映射输出到驱动
        # 电机对象的 forward/backward 也会自动应用极性，所以 PID 不需要再管正反装问题
        if power > 0:
            self.motor.forward(power)
        elif power < 0:
            self.motor.backward(abs(power))
        else:
            self.motor.brake()

    def brake(self):
        """立即刹车锁定"""
        self.target_speed = 0.0
        self.actual_target = 0.0
        self.pid.reset()
        self.motor.brake()


class MotorController:
    def __init__(self):
        """
        三轮 omni 全向底盘中央控制器
        整合了电机驱动引脚与正确的编码器读值引脚，统一正负极性逻辑。
        """
        self.left_node = MotorNode("C28", "C29", "C0", "C1", invert=True)
        self.right_node = MotorNode("C30", "C31", "C2", "C3", invert=False)
        self.head_node = MotorNode("D6", "D7", "D15", "D16", invert=False)

    def set_speeds(self, left, right, head):
        """设定三个轮子的期望脉冲速度"""
        self.left_node.target_speed = float(left)
        self.right_node.target_speed = float(right)
        self.head_node.target_speed = float(head)

    def tick(self):
        """刷新底盘控制时钟 (由 10ms 定时器回调驱动)"""
        self.left_node.update()
        self.right_node.update()
        self.head_node.update()

    def brake_all(self):
        """紧急停止底盘"""
        self.left_node.brake()
        self.right_node.brake()
        self.head_node.brake()