"""
BaseStation.py - 地面基站类
功能
管理无人机、分配任务、接收和处理巡检数据。

"""

from utils import *
from config import *
import random

class BaseStation:
    def __init__(self, base_id, position, coverage_radius):
        self.base_id = base_id
        self.position = position
        self.coverage_radius = coverage_radius  # 覆盖半径
        self.load = 0  # 当前负载
        self.resources = {'trans': 1000, 'comp': 1000}  # 可用资源
        self.price_trans = random.uniform(PRICE_TRANS_BOUND[0], PRICE_TRANS_BOUND[1])  # 传输费用
        self.price_comp = random.uniform(PRICE_COMP_BOUND[0], PRICE_COMP_BOUND[1])  # 计算费用

    def is_in_coverage(self, drone_position):
        distance = calculate_3d_distance(self.position, drone_position)
        return distance <= self.coverage_radius
    
    def receive_data(self, drone_id, data):
        self.data_storage.append((drone_id, data))
        self.process_data(data)

    def process_data(self, data):
        self.tasks.append(data)
        print(f"基站 {self.base_id} 处理: {data}")
        self.tasks.pop(0)
    
    def calculate_utility(self, drones):
        utility = 0
        trans_total = 0
        comp_total = 0
        for drone in drones:
            for task in drone.request_history:
                if task[2] == self.base_id:
                    trans_total += task[0]
                    comp_total += task[1]
        
        utility += self.price_trans * trans_total + self.price_comp * comp_total
        # 惩罚项（假设负数为负载超限，简化处理）
        overload = max(0, self.load - MAX_LOAD)  # 假设最大负载为 5
        utility -= MU * overload
        
        return utility
