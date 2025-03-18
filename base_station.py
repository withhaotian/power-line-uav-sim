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

class BaseStation:
    def __init__(self, position):
        self.position = position
        self.drones = []
        self.tasks = []
        self.data_storage = []

    def assign_task(self, drone):
        if self.tasks:
            task = self.tasks.pop(0)
            task.update_status("assigned")
            return task
        return None

    def receive_data(self, drone_id, data):
        self.data_storage.append((drone_id, data))
        self.process_data(data)

    def process_data(self, data):
        print(f"Processing data: {data}")