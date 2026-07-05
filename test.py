# ============================================================================
# N30电机 100% 满载占空比开环极速测试
# 目的：测量电机在 10ms 周期内能产生的最大物理脉冲数
# 警告：运行前请务必将小车悬空！
# ============================================================================

import time
from machine import Pin, PWM
from smartcar import encoder, ticker

print("==== N30 满载极速测试准备 ====")
print("请确认小车已经悬空！3秒后电机将全速启动...")
time.sleep(3)

# 1. 初始化编码器 (采用你 app_encoder.py 中的引脚配置)
enc_left = encoder("C0", "C1", True)
enc_right = encoder("C2", "C3", False)
enc_head = encoder("D15", "D16", False)

# 2. 定时器滴答标志与回调函数 (10ms精准硬件定时)
ticker_flag = False
def timer_callback_10ms(timer_obj):
    global ticker_flag
    ticker_flag = True

pit = ticker(0)
pit.capture_list(enc_left, enc_right, enc_head)
pit.callback(timer_callback_10ms)
pit.start(10)

# 3. 绕过所有驱动类，直接用底层硬件输出 100% 慢衰减正转电压
# (IN1拉高，IN2 PWM置0，此时电机两端压差最大，全速旋转)
in1_left = Pin("C28", Pin.OUT, value=1)
in2_left = PWM("C29", 13000, duty_u16=0)

in1_right = Pin("C30", Pin.OUT, value=1)
in2_right = PWM("C31", 13000, duty_u16=0)

in1_head = Pin("D6", Pin.OUT, value=1)
in2_head = PWM("D7", 13000, duty_u16=0)

print("==== 测试开始 (按 Ctrl+C 停止) ====")
try:
    last_print_time = time.ticks_ms()
    
    while True:
        if ticker_flag:
            ticker_flag = False
            
            # 获取这 10ms 内的脉冲增量
            left_val = enc_left.get()
            right_val = enc_right.get()
            head_val = enc_head.get()
            
            # 每 100ms 打印一次采样结果
            # 取绝对值 abs() 是因为极性反接的轮子可能读出负数，但我们只关心“最高速度的数值大小”
            if time.ticks_diff(time.ticks_ms(), last_print_time) >= 100:
                last_print_time = time.ticks_ms()
                print("左轮极速: {:<5d} | 右轮极速: {:<5d} | 头轮极速: {:<5d}".format(
                    abs(left_val), abs(right_val), abs(head_val)
                ))
                
except KeyboardInterrupt:
    print("\n[测试结束] 正在紧急刹车...")
finally:
    # 退出时立刻强行刹车 (IN1=1, IN2=100% -> 两端短路刹车)
    in1_left.value(1); in2_left.duty_u16(65535)
    in1_right.value(1); in2_right.duty_u16(65535)
    in1_head.value(1); in2_head.duty_u16(65535)
    
    pit.stop()
    print("电机已停止，安全退出。")