"""
4. BaseStation.py - 地面基站类
功能
管理无人机、分配任务、接收和处理巡检数据。

内容
属性：
position：基站位置（三维坐标）
drones：无人机列表
tasks：任务队列
data_storage：存储接收到的巡检数据
方法：
assign_task(drone)：为无人机分配任务
receive_data(drone_id, data)：接收无人机发送的数据
process_data(data)：处理巡检数据（例如分析视频）
"""

from utils import calculate_distance

class BaseStation:
    def __init__(self, base_id, position, coverage_radius):
        self.base_id = base_id
        self.position = position
        self.drones = []
        self.tasks = []
        self.data_storage = []
        self.coverage_radius = coverage_radius  # 覆盖半径

    def is_in_coverage(self, drone_position):
        distance = calculate_distance(self.position, drone_position)
        return distance <= self.coverage_radius
    
    def receive_data(self, drone_id, data):
        self.data_storage.append((drone_id, data))
        self.process_data(data)

    def process_data(self, data):
        self.tasks.append(data)
        print(f"基站 {self.base_id} 处理: {data}")
        self.tasks.pop(0)