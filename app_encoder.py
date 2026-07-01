# ============================================================================
# 编码器电机驱动层 — PID 控制器 + 单电机封装 + 三路底盘管理
# ============================================================================

import motor
from smartcar import encoder


# ============================================================================
# 位置式 PID 控制器 (通用)
# ============================================================================
class PositionalPID:
    def __init__(self, kp, ki, kd, out_max=100.0):
        self.kp = kp              # 比例系数
        self.ki = ki              # 积分系数
        self.kd = kd              # 微分系数
        self.err = 0              # 当前误差
        self.last_err = 0         # 上一次误差 (用于微分计算)
        self.integral = 0         # 积分累计
        self.out = 0              # 最新输出值
        self.out_max = out_max    # 输出限幅绝对值

    def calc(self, target, current, dt_ms=10):
        """
        计算 PID 输出。
        target:  目标值
        current: 当前实际值
        dt_ms:   距离上次调用的时间间隔 (ms), 默认 10ms
        返回: PID 输出值, 已限幅到 [-out_max, out_max]
        """
        self.err = target - current
        dt_norm = dt_ms / 10.0              # 归一化时间: 以 10ms 为单位

        self.integral += self.err * dt_norm  # 积分累加

        # 积分限幅: 防止积分饱和 (windup)
        if self.integral > 3000: self.integral = 3000
        elif self.integral < -3000: self.integral = -3000

        # 微分项: 误差变化率 (带除零保护)
        derivative = (self.err - self.last_err) / dt_norm if dt_norm > 0 else 0

        # PID 合成
        self.out = (self.kp * self.err) + \
                   (self.ki * self.integral) + \
                   (self.kd * derivative)

        self.last_err = self.err

        # 输出限幅
        if self.out > self.out_max: self.out = self.out_max
        elif self.out < -self.out_max: self.out = -self.out_max

        return self.out


# ============================================================================
# 单电机驱动封装 — 含斜坡规划 + 编码器测速 + 速度 PID + 低通滤波
# ============================================================================
class MotorNode:
    def __init__(self, enc_a, enc_b, in1, in2, invert=False):
        # 编码器 (smartcar 库, 自动累加脉冲)
        self.enc = encoder(enc_a, enc_b, invert)
        self.in1 = in1              # 电机方向引脚
        self.in2 = in2              # 电机 PWM 引脚

        # 速度 PID: 控制实际 RPM 追上目标 RPM
        self.pid = PositionalPID(kp=0.7, ki=0.02, kd=0, out_max=100)

        # 斜坡控制 (Slew Rate Limiting): 平滑起步, 防止电流冲击
        self.target_rpm = 0.0               # 外部设定的终极目标速度
        self.current_target_rpm = 0.0       # PID 当前周期正在追赶的过渡目标速度
        self.accel_step = 13.0              # 每 10ms 允许的最大 RPM 变化量 (= 1300 RPM/s)

        self.current_rpm = 0.0              # 当前实际转速 (经过低通滤波)
        self.current_ticks = 0              # 累计编码器脉冲数

    def update(self):
        """
        每 10ms 由 ChassisController.tick() 调用。
        流程: 斜坡规划 → 编码器测速 → 低通滤波 → 速度 PID → 防抖刹车 → 动力输出
        """
        # ---- 1. 目标速度斜坡规划 ----
        # 将 current_target_rpm 逐步逼近 target_rpm, 限制每步变化不超过 accel_step
        if self.current_target_rpm < self.target_rpm:
            self.current_target_rpm += self.accel_step
            if self.current_target_rpm > self.target_rpm:
                self.current_target_rpm = self.target_rpm

        elif self.current_target_rpm > self.target_rpm:
            self.current_target_rpm -= self.accel_step
            if self.current_target_rpm < self.target_rpm:
                self.current_target_rpm = self.target_rpm

        # ---- 2. 编码器测速 ----
        dticks = self.enc.get()                     # 读取 10ms 内的脉冲增量
        self.current_ticks += dticks                # 累加总脉冲
        if self.current_ticks > 950:                # 限幅
            self.current_ticks = 950
        elif self.current_ticks < -950:
            self.current_ticks = -950
        raw_rpm = dticks * 17.14                    # 脉冲增量 → 瞬时 RPM 换算

        # ---- 3. 低通滤波 (EMA, α=0.15) ----
        self.current_rpm = self.current_rpm * 0.85 + raw_rpm * 0.15

        # ---- 4. 速度 PID ----
        power = self.pid.calc(self.current_target_rpm, self.current_rpm)

        # ---- 5. 防抖刹车: 目标为 0 且实际转速 < 5 RPM → 强制硬件刹车 ----
        if self.target_rpm == 0 and abs(self.current_rpm) < 5:
            motor.brake(self.in1, self.in2)
            self.pid.out = 0
            self.pid.integral = 0
            self.current_target_rpm = 0             # 清零过渡目标, 准备下次起步
            return

        # ---- 6. 动力输出 ----
        if power >= 0:
            motor.forward(self.in1, self.in2, power)
        else:
            motor.backward(self.in1, self.in2, abs(power))


# ============================================================================
# 三路底盘控制器
# ============================================================================
class ChassisController:
    def __init__(self):
        # 三个电机节点: 右轮 / 左轮 / 尾轮
        self.right_node = MotorNode("C2", "C3", motor.right_in1, motor.right_in2)
        self.left_node = MotorNode("C0", "C1", motor.left_in1, motor.left_in2)
        self.head_node = MotorNode("D15", "D16", motor.head_in1, motor.head_in2)

    def set_speeds(self, left_rpm, right_rpm, head_rpm):
        """设置三路电机的目标转速 (RPM), 斜坡规划自动平滑过渡"""
        self.left_node.target_rpm = left_rpm
        self.right_node.target_rpm = right_rpm
        self.head_node.target_rpm = head_rpm

    def brake_all(self):
        """三路同时刹车 (目标转速设 0 + 硬件制动)"""
        self.set_speeds(0, 0, 0)
        motor.brake(motor.left_in1, motor.left_in2)
        motor.brake(motor.right_in1, motor.right_in2)
        motor.brake(motor.head_in1, motor.head_in2)

    def tick(self):
        """每帧调用: 驱动三路 MotorNode 更新 (测速 + PID + 输出)"""
        self.left_node.update()
        self.right_node.update()
        self.head_node.update()
