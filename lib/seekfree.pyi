"""
Type stub for the seekfree module (built into RT1021 firmware).
IDE autocomplete only -- not uploaded to the board.
"""

from typing import Any

class KEY_HANDLER:
    def __init__(self, threshold: int = 10) -> None: ...
    def get(self) -> Any: ...
    def clear(self, key_id: int) -> None: ...

class IMU660RX:
    TYPE_RC: int
    RATE_120HZ: int

    def __init__(self, imu_type: int, quar_rate: int) -> None: ...
    def get_euler(self) -> list[float]: ...
