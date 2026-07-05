# ============================================================================
# 串口通信层 — UART 封装 + 简易指令解析
# ============================================================================

from machine import UART


class Serial:
    def __init__(self, uart_id, baudrate=115200):
        self.uart = UART(uart_id, baudrate=baudrate)

    def send_msg(self, text_data):
        msg = str(text_data) + '\n'
        self.uart.write(msg.encode('utf-8'))

    def read_msg(self):
        if self.uart.any():
            line_bytes = self.uart.readline()
            if line_bytes:
                try:
                    return line_bytes.decode('utf-8').strip()
                except UnicodeError:
                    return None
        return None
    
    def send_motor_status(self, app):
        """专门用于发送电机的目标编码器速度(T)和真实反馈速度(R)"""
        tl = app.motor.left_node.target_speed
        rl = app.motor.left_node.current_speed
        tr = app.motor.right_node.target_speed
        rr = app.motor.right_node.current_speed
        th = app.motor.head_node.target_speed
        rh = app.motor.head_node.current_speed

        # 获取当前的测试模式状态，兼容单轮测试(-1代表全向正常模式)
        test_idx = getattr(app, 'test_motor_idx', -1)

        if test_idx == 0:
            self.send_msg("{:.0f}, {:.0f}".format(tl, rl))
        elif test_idx == 1:
            self.send_msg("{:.0f}, {:.0f}".format(tr, rr))
        elif test_idx == 2:
            self.send_msg("{:.0f}, {:.0f}".format(th, rh))
        else:
            # 紧凑格式发送三个轮子的数据，防止单包数据过长
            self.send_msg("L(T{:.0f} R{:.0f}) R(T{:.0f} R{:.0f}) H(T{:.0f} R{:.0f})".format(tl, rl, tr, rr, th, rh))


def process_received_data(data_str, app):
    """解析串口指令, 返回 True 表示已处理"""
    if not data_str:
        return False

    data_str = data_str.replace('\x00', '').replace('\r', '').replace('\n', '').strip()

    if ":" in data_str:
        try:
            key_part, value_str = data_str.split(":", 1)
            key = key_part.strip().upper()
            value_str = value_str.strip()

            # ==========================================================
            # 1. 动态调节电机速度环 PID 参数
            # 格式: PID:轮子,Kp,Ki,Kd
            # 例子: PID:ALL,0.03,0.002,0.01 (全部轮子设置)
            #       PID:L,0.05,0.001,0.02   (仅设置左轮)
            # ==========================================================
            if key == "PID":
                parts = value_str.split(",")
                if len(parts) == 4:
                    target = parts[0].strip().upper()
                    kp = float(parts[1])
                    ki = float(parts[2])
                    kd = float(parts[3])
                    
                    nodes = []
                    if target == "L": nodes.append(app.motor.left_node)
                    elif target == "R": nodes.append(app.motor.right_node)
                    elif target == "H": nodes.append(app.motor.head_node)
                    elif target == "ALL": nodes = [app.motor.left_node, app.motor.right_node, app.motor.head_node]
                    
                    for node in nodes:
                        node.pid.kp = kp
                        node.pid.ki = ki
                        node.pid.kd = kd
                        node.pid.reset()  # 更改参数后清空积分项，防止系统出现突然猛转的冲击
                        
                    return True

            # ==========================================================
            # 2. 独立电机阶跃测速 (用于观察 PID 曲线)
            # 格式: SPD:轮子,占空比
            # 例子: SPD:L,50 (给左轮50%的占空比意图)
            #       SPD:ALL,30 (三个轮子同时30%)
            # ==========================================================
            elif key == "SPD":
                parts = value_str.split(",")
                if len(parts) == 2:
                    target = parts[0].strip().upper()
                    duty = float(parts[1])
                    
                    app.serial_debug = True   # 进入串口独立调试模式
                    app.is_testing = False    # 挂起按键测试模式
                    
                    if target == "L": app.serial_duties[0] = duty
                    elif target == "R": app.serial_duties[1] = duty
                    elif target == "H": app.serial_duties[2] = duty
                    elif target == "ALL": app.serial_duties = [duty, duty, duty]
                    return True

        except Exception:
            return False

    if data_str.upper() == "STOP":
        if hasattr(app, 'set_brake'):
            app.set_brake()
        return True

    return False
