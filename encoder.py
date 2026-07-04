# ============================================================================
# 编码器脉冲读值极简测试程序 (精确 10ms 硬件定时采样)
# ============================================================================

import time
from smartcar import encoder, ticker

# 1. 实例化三个编码器 (基于你小车原本在 app_encoder.py 中配置的物理引脚)
# 左轮编码器引脚: C0, C1 (由于物理安装方向相反，设置 invert=True 统一极性)
enc_left = encoder("C0", "C1", True)

# 右轮编码器引脚: C2, C3
enc_right = encoder("C2", "C3", False)

# 头轮/尾轮编码器引脚: D15, D16
enc_head = encoder("D15", "D16", False)


# 2. 定时器滴答标志与回调函数
ticker_flag = False

def timer_callback_10ms(timer_obj):
    """
    10ms 定时中断回调函数
    """
    global ticker_flag
    ticker_flag = True


# 3. 初始化并配置 10ms Ticker 定时器
pit = ticker(0)

# 关键：必须将编码器对象加入 capture_list 中！
# 这样底层硬件中断每 10ms 会在精准时刻自动调用各个编码器的 capture 读数并清空寄存器，保证不丢脉冲
pit.capture_list(enc_left, enc_right, enc_head)
pit.callback(timer_callback_10ms)
pit.start(10)  # 启动 10ms 精确时钟中断


print("==== 编码器 10ms 实时读值测试开始 ====")
print("注意：请手动用手旋转小车的轮子，观察三个轮子的脉冲输出。")
print("前进时读数应当为正，后退时读数应当为负。")
print("按下 Ctrl+C 键可以安全停止测试。\n")

try:
    last_print_time = time.ticks_ms()
    
    while True:
        if ticker_flag:
            ticker_flag = False
            
            # 只有当 Ticker 定时器触发后，底层才刷新了数据，此时读取才是精确的 10ms 增量
            # get() 获取的是这 10ms 周期内的脉冲计数值
            left_val = enc_left.get()
            right_val = enc_right.get()
            head_val = enc_head.get()
            
            # 为了防止控制台刷新过快导致卡顿，我们每 100ms 在屏幕上打印一次
            if time.ticks_diff(time.ticks_ms(), last_print_time) >= 100:
                last_print_time = time.ticks_ms()
                print("左轮脉冲: {:<5d} | 右轮脉冲: {:<5d} | 头轮脉冲: {:<5d}".format(left_val, right_val, head_val))
                
except KeyboardInterrupt:
    print("\n[安全防护] 捕获用户中断信号，测试结束。")
finally:
    # 退出程序时，务必关闭定时器释放硬件资源
    pit.stop()
    print("Ticker 定时器已安全关闭。")