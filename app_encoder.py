from seekfree import encoder
from drv8871_simple import DRV8871Motor

class PositionalPID:
    def __init__(self, kp, ki, kd, out_max=300.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.integral = 0.0
        self.last_err = 0.0

    def calc(self, err):
        derivative = err - self.last_err
        self.last_err = err

        temp_integral = self.integral + err
        p_term = self.kp * err
        i_term = self.ki * temp_integral
        d_term = self.kd * derivative
        total_out = p_term + i_term + d_term

        if abs(total_out) >= self.out_max and (total_out * err > 0):
            pass
        else:
            self.integral = temp_integral

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


class MotorNode:
    def __init__(self, in1, in2, enc_id, invert=False, kp=1.6, ki=0.05, kd=0.03, ramp_limit=12.0):
        self.motor = DRV8871Motor(in1, in2, invert=invert)
        self.enc = encoder(enc_id)
        self.pid = PositionalPID(kp, ki, kd, out_max=300.0) # 默认上限设为 300
        
        self.invert = invert
        self.target_speed = 0.0      # 上层下发的目标测速 (单位：脉冲数/10ms)
        self.current_speed = 0.0     # 校正后的实际测速 (单位：脉冲数/10ms)
        self.actual_target = 0.0     # 经过斜坡过渡后的实际目标值
        self.ramp_limit = ramp_limit

    def update(self):
        """
        核心周期轮询函数 (每 10ms 稳定调用一次)
        """
        raw_pulses = self.enc.get()
        self.current_speed = -raw_pulses if self.invert else raw_pulses

        if self.target_speed == 0.0 and abs(self.current_speed) < 1.5:
            self.brake()
            return

        diff = self.target_speed - self.actual_target
        if abs(diff) <= self.ramp_limit:
            self.actual_target = self.target_speed
        else:
            self.actual_target += self.ramp_limit if diff > 0 else -self.ramp_limit

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

class ChassisController:
    def __init__(self):
        self.left_node = MotorNode("C28", "C29", 0, invert=True)
        self.right_node = MotorNode("C30", "C31", 1, invert=False)
        self.head_node = MotorNode("D6", "D7", 2, invert=False)

    def set_speeds(self, left, right, head):
        """
        设定三个轮子的期望脉冲速度
        """
        self.left_node.target_speed = float(left)
        self.right_node.target_speed = float(right)
        self.head_node.target_speed = float(head)

    def tick(self):
        """
        刷新底盘控制时钟 (由 10ms 定时器回调驱动)
        """
        self.left_node.update()
        self.right_node.update()
        self.head_node.update()

    def brake_all(self):
        self.left_node.brake()
        self.right_node.brake()
        self.head_node.brake()
