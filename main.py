# ============================================================================
# salve_car — 精简机器人固件
# 硬件: 三路全向轮 + 编码器 + IMU + 按键 + 双串口
# ============================================================================

import gc
import time
from machine import Pin
from smartcar import ticker
from seekfree import KEY_HANDLER, IMU660RX
from controller import AppController
from app_imu660rx import Imu660Cal
from app_uart import Serial, process_received_data
# ============================================================================
# 硬件初始化
# ============================================================================
buzzer = Pin("C18", Pin.OUT)
led = Pin("C4", Pin.OUT)
light = Pin("B27", Pin.OUT)

key = KEY_HANDLER(10)
imu_raw = IMU660RX(imu_type=IMU660RX.TYPE_RC, quar_rate=IMU660RX.RATE_120HZ)
imu = Imu660Cal(imu_raw)

app = AppController(key)

camera = Serial(3, 115200)
lora = Serial(1, 115200)

# ============================================================================
# 10ms 定时器
# ============================================================================
ticker_flag = False


def timer_callback_10ms(timer):
    global ticker_flag
    ticker_flag = True


pit1 = ticker(0)
pit1.capture_list(key, app.chassis.left_node.enc,
                  app.chassis.right_node.enc,
                  app.chassis.head_node.enc)
pit1.callback(timer_callback_10ms)
pit1.start(10)

# ============================================================================
# 主循环
# ============================================================================


def main():
    global ticker_flag

    start_time = time.ticks_ms()
    init_done = False
    light.value(1)

    lora.send_msg("salve_car ready!")

    buzzer_timer = 0
    led_tick = 0

    while True:
        if ticker_flag:
            ticker_flag = False

            yaw = imu.update()
            nowtime = time.ticks_ms()

            # 上电 1s 后校准 IMU 零角度
            if not init_done:
                if time.ticks_diff(nowtime, start_time) >= 1000:
                    imu.set_yaw_zero()
                    init_done = True
                continue

            # ---- LED 心跳 ----
            led_tick += 1
            if led_tick >= 200:
                led_tick = 0
            led.value(1 if led_tick < 100 else 0)

            # ---- 蜂鸣器 ----
            if buzzer_timer > 0:
                buzzer.value(1)
                buzzer_timer -= 1
            else:
                buzzer.value(0)

            # ---- 按键处理 ----
            app.keyprocess(yaw)

            if app.trigger_mission:
                app.trigger_mission = False
                buzzer_timer = 10
                # 用户可在 KEY0 短按时自定义行为

            if app.long_press_action is not None:
                camera.send_msg(app.long_press_action)
                lora.send_msg("KEY:" + app.long_press_action)
                app.long_press_action = None

            # ---- 串口指令 ----
            cam_data = camera.read_msg()
            if cam_data:
                lora.send_msg(cam_data)  # 转发摄像头数据

            lora_data = lora.read_msg()
            if lora_data:
                result = process_received_data(lora_data, app)
                if result:
                    lora.send_msg("OK")

            # ---- 运动控制 ----
            app.control()

            gc.collect()


if __name__ == '__main__':
    main()
