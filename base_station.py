"""
BaseStation.py - 地面基站类
功能
管理无人机、分配任务、接收和处理巡检数据。

"""

from utils import *
from config import *

class BaseStation:
    def __init__(self, base_id, position, coverage_radius):
        self.base_id = base_id
        self.position = position
        self.drones = []
        self.tasks = []
        self.data_storage = []
        self.coverage_radius = coverage_radius  # 覆盖半径
        self.load = 0.0  # 当前负载
        self.resources = {'trans': 1000, 'comp': 1000}  # 可用资源

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
    
    def calculate_utility(self):
        utility = 0
        for drone in self.drones:
            z_jk = 1  # 无人机已分配到此基站
            utility += (P_TRANS * drone.trans_demand + P_COMP * drone.comp_demand) * z_jk
        # 惩罚项（假设负数为负载超限，简化处理）
        overload = max(0, len(self.drones) - MAX_LOAD)  # 假设最大负载为 5
        utility -= MU * overload
        return utility

    def assign_task(self, drone):
        # 基于效用和成本优化选择任务
        best_task, max_utility = None, float('-inf')
        for task in self.tasks:
            drone.task = task  # 临时分配
            utility = self.calculate_utility() - drone.calculate_cost(self)
            if utility > max_utility:
                max_utility = utility
                best_task = task
        drone.task = best_task
        return best_task
