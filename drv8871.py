# ============================================================================
# DRV8871DDAR 电机驱动芯片 — 三轮开环最简底层驱动类
# ============================================================================

from machine import Pin, PWM

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

    def coast(self):
        self.in1.value(0)
        self.in2.duty_u16(0)