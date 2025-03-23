"""
Drone.py - 无人机类
功能
定义无人机的属性和行为，模拟无人机在巡检任务中的操作。
"""

from utils import *
from config import *

class Drone:
    def __init__(self, drone_id, position, speed, battery_capacity):
        self.drone_id = drone_id
        self.position = position  # [x, y, z]
        self.speed = speed
        self.battery = battery_capacity
        self.base_station = None
        self.position_history = None
        self.trans_demand = 10  # 传输需求
        self.comp_demand = 20   # 计算需求  

    def move_to(self, target_position):
        # 计算水平距离 d_xy 和高度变化 |Δz|
        d_xy = calculate_2d_distance(self.position, target_position)  # 现有函数计算二维距离
        delta_z = abs(target_position[2] - self.position[2])  # z 轴高度差
        energy_consumed = ALPHA * d_xy + BETA * delta_z  # 根据公式计算能耗
        self.battery -= energy_consumed  # 更新电量
        self.position = target_position
        return d_xy

    def inspect(self, power_station):
        if self.position == power_station.position:
            data = f"来自电站 {power_station.station_id} 的4K视频数据"
            self.battery -= 5  # 巡检固定耗电
            return data
        return None

    def send_data(self, base_station, data):
        base_station.receive_data(self.drone_id, data)
    
    def calculate_cost(self, base_stations):
        """计算无人机成本"""
        if not self.position_history or len(self.position_history) < 2:
            return 0
        cost = 0
        # 服务成本
        for bs in base_stations:
            if bs == self.base_station:
                cost += (P_TRANS * self.trans_demand + P_COMP * self.comp_demand)
        # 能耗成本
        if self.position_history:
            for i in range(len(self.position_history) - 1):
                v_a, v_b = self.position_history[i], self.position_history[i + 1]
                d_ab = calculate_2d_distance(v_a, v_b)
                z_diff = abs(v_a[2] - v_b[2])
                cost += ALPHA * d_ab + BETA * z_diff
        return cost

    def dynamic_offload(self, base_stations):
        """动态卸载决策"""
        min_cost = float('inf')
        best_bs = None
        for bs in base_stations:
            if bs.is_in_coverage(self.position):
                self.base_station = bs
                cost = self.calculate_cost(base_stations)
                if cost < min_cost:
                    min_cost = cost
                    best_bs = bs
        self.base_station = best_bs
        return best_bs
