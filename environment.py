"""
5. Environment.py - 三维空间环境类
功能
模拟三维空间，管理所有实体之间的交互。

内容
属性：
size：空间尺寸（例如 [x_max, y_max, z_max]）
drones：无人机列表
power_stations：电站列表
base_station：地面基站
方法：
initialize()：初始化环境
update()：更新环境状态（例如无人机位置、任务状态）
"""

from base_station import BaseStation
from drone import Drone
from power_station import PowerStation
from utils import random_position
from config import DRONE_SPEED, BATTERY_CAPACITY
from task import Task

class Environment:
    def __init__(self, size):
        self.size = size  # [x_max, y_max, z_max]
        self.drones = []
        self.power_stations = []
        self.base_station = None

    def initialize(self, num_drones, num_stations):
        self.base_station = BaseStation([0, 0, 0])
        for i in range(num_drones):
            drone = Drone(i, [0, 0, 0], DRONE_SPEED, BATTERY_CAPACITY)
            self.drones.append(drone)
            self.base_station.drones.append(drone)
        for i in range(num_stations):
            pos = random_position(self.size)
            station = PowerStation(i, pos)
            self.power_stations.append(station)
            self.base_station.tasks.append(Task(i, station))

    def update(self):
        for drone in self.drones:
            if not drone.task:
                drone.request_task(self.base_station)
            if drone.task and drone.task.status == "assigned":
                drone.move_to(drone.task.target.position)
                data = drone.inspect(drone.task.target)
                if data:
                    drone.send_data(self.base_station, data)
                    drone.task.update_status("completed")
                    drone.task = None