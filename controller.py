# ============================================================================
# 运动控制器 — 极简全向运动学解算 + 按键单轮调试 + 串口独立调速
# ============================================================================
import app_encoder

class AppController:
    def __init__(self, key_handler):
        # ---- 按键相关 ----
        self.key = key_handler
        self.key_data = self.key.get()
        self.long_press_keyflag = [False, False]
        self.long_press_action = None
        self.trigger_mission = False

        # ---- 底盘电机闭环控制中心 ----
        self.motor = app_encoder.MotorController()

        # ---- 运动学核心状态 ----
        self.vx = 0.0  
        self.vy = 0.0  
        self.vz = 0.0  

        # ---- 按键单轮测试状态 ----
        self.test_motor_idx = -1  # -1:正常全向, 0:左轮, 1:右轮, 2:头轮
        self.test_duty = 20.0     

        # ---- 串口独立调速状态 ----
        self.serial_debug = False 
        self.serial_duties = [0.0, 0.0, 0.0] # L, R, H 独立目标占空比

    def set_brake(self):
        """
        紧急刹车，全向速度归零并强制退出所有测试/调试模式。
        当收到串口 "STOP" 指令或长按 KEY1 时，该方法会被调用。
        """
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        # 清除所有调试状态
        self.test_motor_idx = -1
        self.serial_debug = False
        self.serial_duties = [0.0, 0.0, 0.0]
        self.trigger_mission = False
        
        self.motor.brake_all()
        print("[Controller] Brake Activated! All modes reset.")

    def update_kinematics(self):
        """全向运动学解算核心 (严格遵循逐飞科技矩阵模型)"""
        Vcx = self.vx
        Vcy = self.vy
        Vz = self.vz

        SQRT3_3 = 0.577350
        ONE_THIRD = 0.333333

        v_left = (ONE_THIRD * Vcx) + (SQRT3_3 * Vcy) + (ONE_THIRD * Vz)
        v_right = (ONE_THIRD * Vcx) - (SQRT3_3 * Vcy) + (ONE_THIRD * Vz)
        v_tail = -(2.0 / 3.0 * Vcx) + (ONE_THIRD * Vz)

        max_v = max(abs(v_left), abs(v_right), abs(v_tail))
        if max_v > 100.0:
            scale = 100.0 / max_v
            v_left *= scale
            v_right *= scale
            v_tail *= scale

        self.motor.set_speeds(v_left, v_right, v_tail)

    def keyprocess(self, yaw=0.0):
        """按键处理逻辑"""
        self.key_data = self.key.get()

        for i in range(2):
            if self.key_data[i] == 1:
                self.key.clear(i + 1)
                
                # ---- 单击按键 1 (KEY1): 轮询切换测试电机 ----
                if i == 0:
                    self.trigger_mission = True
                    self.serial_debug = False # 优先响应按键，立刻覆盖串口调试模式
                    
                    self.test_duty = 20.0
                    self.test_motor_idx = (self.test_motor_idx + 1) % 3
                    
                    motor_names = ["Left", "Right", "Head"]
                    print(f"[Controller] Testing Motor: {motor_names[self.test_motor_idx]} at 20%")
                    
                # ---- 单击按键 2 (KEY2): 当前测试电机占空比增加 10% ----
                elif i == 1:
                    if self.test_motor_idx != -1:
                        self.test_duty += 10.0
                        if self.test_duty > 100.0:
                            self.test_duty = 100.0
                        print(f"[Controller] Speed up! Current duty: {self.test_duty}%")

            elif self.key_data[i] == 2:
                if not self.long_press_keyflag[i]:
                    self.long_press_keyflag[i] = True
                    
                    # ---- 长按按键 1 (KEY1): 立即停止并刹车 ----
                    if i == 0:
                        self.set_brake()
                        self.long_press_action = 'r'
                        
                    elif i == 1:
                        self.long_press_action = 'b'

            elif self.key_data[i] == 0:
                self.long_press_keyflag[i] = False

    def control(self):
        """控制路由器 (每 10ms 被主循环精准调用一次)"""
        
        # 第一步：判断当前模式并下发目标轮速
        if self.serial_debug:
            # 1. 串口独立调速模式
            self.motor.set_speeds(self.serial_duties[0], self.serial_duties[1], self.serial_duties[2])
        elif self.test_motor_idx != -1:
            # 2. 按键单轮测试模式
            v_left  = self.test_duty if self.test_motor_idx == 0 else 0.0
            v_right = self.test_duty if self.test_motor_idx == 1 else 0.0
            v_head  = self.test_duty if self.test_motor_idx == 2 else 0.0
            self.motor.set_speeds(v_left, v_right, v_head)
        else:
            # 3. 正常模式：使用运动学矩阵计算轮速
            self.update_kinematics()
        
        # 第二步：驱动底层 3 个轮子的 PID 控制器开始控速
        self.motor.tick()