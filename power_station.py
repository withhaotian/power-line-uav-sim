"""
PowerStation.py - 电站类
功能
定义电站的属性，作为无人机的巡检目标。
"""

class PowerStation:
    def __init__(self, station_id, position):
        self.station_id = station_id
        self.position = position  # [x, y, z]
        self.inspection_needed = True

