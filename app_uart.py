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

            # FREE: 自由平移 (速度,角度)
            if key == "FREE":
                parts = value_str.split(",")
                if len(parts) == 2:
                    speed = float(parts[0])
                    angle = float(parts[1])
                    app.set_free_move(
                        current_yaw=app.current_yaw,
                        speed=speed,
                        angle_deg=angle,
                        relative_heading=0.0
                    )
                    return True

            # TURN: 原地转向 (相对角度)
            elif key == "TURN":
                ang_val = float(value_str)
                app.move_mode = 'IDLE'
                app.is_tracking = False
                app.chassis.brake_all()
                app.turn_to_angle(current_yaw=app.current_yaw, relative_angle=ang_val)
                return True

            # TRACK: 直线循迹开关 (1=开, 0=关)
            elif key == "TRACK":
                val = int(value_str)
                if val == 1:
                    app.move_mode = 'IDLE'
                    app.is_tracking = True
                    app.chassis.brake_all()
                else:
                    app.is_tracking = False
                    app.chassis.brake_all()
                return True

            # SET: 参数热更新 (变量名=值)
            elif key == "SET":
                pairs = value_str.split(",")
                changed = False
                for pair in pairs:
                    if "=" in pair:
                        attr_name, attr_val = pair.split("=")
                        attr_name = attr_name.strip().lower()
                        try:
                            val = float(attr_val.strip())
                            if hasattr(app, attr_name):
                                original = getattr(app, attr_name)
                                if isinstance(original, (int, float)):
                                    setattr(app, attr_name, val)
                                    changed = True
                        except ValueError:
                            continue
                return changed

        except Exception:
            return False

    # STOP: 紧急刹车
    if data_str.upper() == "STOP":
        app.move_mode = 'IDLE'
        app.is_tracking = False
        app.chassis.brake_all()
        return True

    return False
