"""
3. PowerStation.py - 电站类
功能
定义电站的属性，作为无人机的巡检目标。

内容
属性：
station_id：电站ID
position：位置（三维坐标 [x, y, z]）
inspection_needed：是否需要巡检（布尔值）
"""

class PowerStation:
    def __init__(self, station_id, position):
        self.station_id = station_id
        self.position = position  # [x, y, z]
        self.inspection_needed = True

        