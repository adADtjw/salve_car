# ============================================================================
# 运动控制器 — PID + 运动学解算 + 按键处理
# ============================================================================

import app_encoder
import time
import math


def _norm(angle):
    while angle > 180.0: angle -= 360.0
    while angle < -180.0: angle += 360.0
    return angle


class AppController:
    def __init__(self, key_handler):
        self.key = key_handler
        self.key_data = self.key.get()
        self.long_press_keyflag = [False, False]
        self.long_press_action = None

        # 底盘 (三路电机 + 编码器)
        self.chassis = app_encoder.ChassisController()

        # ---- PID 控制器 (3个) ----
        self.yaw_pid = app_encoder.PositionalPID(kp=6.0, ki=0.0, kd=0.0, out_max=120.0)
        self.track_pid = app_encoder.PositionalPID(kp=4.2, ki=0.3, kd=10.0, out_max=250.0)
        self.free_pid = app_encoder.PositionalPID(kp=5.0, ki=0.0, kd=0.5, out_max=250.0)

        # ---- 陀螺仪相关 ----
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.is_turning = False
        self.turn_ok = False
        self.turn_count = 0
        self.is_tracking = False
        self.move_mode = 'IDLE'           # IDLE / FREE

        # ---- 平移速度 (机器人坐标系) ----
        self.vx = 0.0
        self.vy = 0.0

        # ---- 自由平移参数 ----
        self.free_speed = 200.0
        self.free_angle = 90.0
        self.free_relative_heading = 0.0
        self.is_waiting_turn = False
        self.track_base_speed = 250.0
        self.action_start_time = 0

        # ---- 按键触发标志 ----
        self.trigger_mission = False

    # ================================================================
    # 轻量复位
    # ================================================================
    def soft_reset(self):
        self.vx = self.vy = 0.0
        self.turn_ok = self.is_waiting_turn = self.is_turning = False
        for p in (self.yaw_pid, self.track_pid, self.free_pid):
            p.integral = 0.0
            p.last_err = 0.0

    # ================================================================
    # 原地相对旋转
    # ================================================================
    def turn_to_angle(self, current_yaw, relative_angle=None):
        if relative_angle is not None:
            relative_angle = _norm(relative_angle)
            target = _norm(current_yaw + relative_angle)
            self.target_yaw = target
            self.is_turning = True
            self.turn_ok = False
            self.yaw_pid.integral = 0
            self.yaw_pid.last_err = 0
            self.turn_count = 0
            return

        if self.is_turning:
            error = self.target_yaw - current_yaw
            if error > 180.0: error -= 360.0
            elif error < -180.0: error += 360.0

            if abs(error) <= 1.0:
                self.chassis.brake_all()
                self.turn_count += 1
                if self.turn_count >= 15:
                    self.is_turning = False
                    self.turn_ok = True
                    self.turn_count = 0
                return
            else:
                self.turn_count = 0

            spin_rpm = self.yaw_pid.calc(error, 0.0)
            if spin_rpm > 200.0: spin_rpm = 200.0
            elif spin_rpm < -200.0: spin_rpm = -200.0

            self.chassis.set_speeds(spin_rpm, -spin_rpm, -spin_rpm)

    # ================================================================
    # 直线循迹 — 陀螺仪 yaw 反馈差速纠偏
    # ================================================================
    def track_straight(self, current_yaw, base_rpm=150.0):
        if not self.is_tracking:
            return

        error = self.target_yaw - current_yaw
        if error > 180.0: error -= 360.0
        elif error < -180.0: error += 360.0

        spin_rpm = self.track_pid.calc(error, 0.0)
        if spin_rpm > 100.0: spin_rpm = 100.0
        elif spin_rpm < -100.0: spin_rpm = -100.0

        left_rpm = base_rpm + spin_rpm
        right_rpm = base_rpm - spin_rpm
        head_rpm = 0.0 - spin_rpm

        self.chassis.set_speeds(left_rpm, right_rpm, head_rpm)

    # ================================================================
    # 自由平移
    # ================================================================
    def set_free_move(self, current_yaw, speed, angle_deg, relative_heading=0.0):
        self.move_mode = 'FREE'
        self.turn_to_angle(current_yaw, relative_angle=relative_heading)
        self.is_waiting_turn = True

        angle_rad = math.radians(angle_deg)
        self.vx = speed * math.sin(angle_rad)
        self.vy = speed * math.cos(angle_rad)

    # ================================================================
    # 紧急刹车
    # ================================================================
    def set_brake(self):
        self.move_mode = 'IDLE'
        self.vx = 0.0
        self.vy = 0.0
        self.chassis.brake_all()

    # ================================================================
    # 运动学解算 — vx/vy → 三轮 RPM
    # ================================================================
    def update_kinematics(self, current_yaw):
        if self.move_mode == 'IDLE':
            return

        # FREE 模式等待转向完成
        if self.move_mode == 'FREE' and self.is_waiting_turn:
            self.turn_to_angle(current_yaw)
            if self.turn_ok:
                self.turn_ok = False
                self.is_waiting_turn = False
                self.free_pid.integral = 0
                self.free_pid.last_err = 0
                self.action_start_time = time.ticks_ms()
            return

        # yaw 闭环纠偏
        error = self.target_yaw - current_yaw
        error = _norm(error)

        pid_spin = self.free_pid.calc(error, 0.0)
        if pid_spin > 300.0: pid_spin = 300.0
        elif pid_spin < -300.0: pid_spin = -300.0

        # 全向轮运动学: 三轮 120° 分布
        SQRT3_2 = 0.866025
        base_tail = self.vx * 1.0 + self.vy * 0.0
        base_right = self.vx * -0.5 + self.vy * SQRT3_2
        base_left = self.vx * 0.5 + self.vy * SQRT3_2

        # 前轮差速纠偏 + 尾轮让权
        v_right = base_right - pid_spin
        v_left = base_left + pid_spin

        if base_tail > 0:
            v_tail = max(0, base_tail)
        elif base_tail < 0:
            v_tail = min(0, base_tail)
        else:
            v_tail = 0.0

        self.chassis.set_speeds(v_left, v_right, v_tail)

    # ================================================================
    # 按键处理
    # ================================================================
    def keyprocess(self, yaw):
        self.key_data = self.key.get()
        self.current_yaw = yaw

        for i in range(2):
            if self.key_data[i] == 1:
                self.key.clear(i + 1)
                if i == 0:
                    self.trigger_mission = True
                    self.chassis.set_speeds(200,200,200)

            elif self.key_data[i] == 2:
                if not self.long_press_keyflag[i]:
                    self.long_press_keyflag[i] = True
                    if i == 0:
                        self.long_press_action = 'r'
                    elif i == 1:
                        self.long_press_action = 'b'

            elif self.key_data[i] == 0:
                self.long_press_keyflag[i] = False

    # ================================================================
    # 控制路由器
    # ================================================================
    def control(self):
        if self.is_turning and self.move_mode == 'IDLE':
            self.turn_to_angle(self.current_yaw)
        elif self.is_tracking and self.move_mode == 'IDLE' and not self.is_turning:
            self.track_straight(self.current_yaw, base_rpm=self.track_base_speed)
        else:
            self.update_kinematics(self.current_yaw)

        self.chassis.tick()
