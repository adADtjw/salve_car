# ============================================================================
# IMU660RX 陀螺仪封装 — 零偏校准 + 角度归一化 + 相对偏航角输出
# ============================================================================

class Imu660Cal:
    def __init__(self, imu_obj):
        self.imu = imu_obj
        self.yaw_offset = 0.0                     # 零偏角度 (set_yaw_zero 时记录)
        # 预获取 euler 数据引用, 将 Python 对象与硬件缓冲区链接
        self.euler_data = self.imu.get_euler()

    def set_yaw_zero(self):
        """
        将当前车头朝向定义为相对 0°。
        记录当前绝对角度作为零偏, 后续所有 update() 返回值均扣除该零偏。
        """
        raw_start_yaw = self.euler_data[2]        # euler_data[2] = Yaw 角
        self.yaw_offset = self.normalize_angle(raw_start_yaw)

    def normalize_angle(self, angle):
        """将角度归一化到 [-180°, 180°] 范围"""
        while angle > 180.0: angle -= 360.0
        while angle < -180.0: angle += 360.0
        return angle

    def update(self):
        """
        每帧调用, 返回当前相对于零偏的偏航角。
        返回值范围: [-180°, 180°]
        """
        raw_yaw = self.euler_data[2]              # 读取硬件缓冲区的原始 Yaw
        relative_yaw = self.normalize_angle(raw_yaw - self.yaw_offset)
        return relative_yaw
