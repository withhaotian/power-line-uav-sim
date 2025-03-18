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
from utils import random_position, calculate_distance
from config import *
from task import Task

class Environment:
    def __init__(self, size):
        """
        初始化三维空间环境。
        
        参数:
            size: 空间尺寸，例如 [x_max, y_max, z_max]
        """
        self.size = size  # [x_max, y_max, z_max]
        self.drones = []  # 无人机列表
        self.power_stations = []  # 电站列表
        self.base_stations = []  # 地面基站列表

    def initialize(self, num_drones, num_stations, num_base_stations):
        """
        初始化环境，创建基站、无人机和电站，并进行分配。
        
        参数:
            num_drones: 无人机数量
            num_stations: 电站数量
            num_base_stations: 基站数量
        """
        # 创建多个地面基站
        for i in range(num_base_stations):
            x = (i + 0.5) * (self.size[0] / num_base_stations)  # x_max = self.size[0]
            y = self.size[1] / 2                                # y_max = self.size[1]
            pos = [x, y, HEIGHT_BASE_STATION]
            base = BaseStation(i, pos, COVERAGE_RADIUS)         # 创建基站对象
            self.base_stations.append(base)                     # 添加到基站列表
        
        # 创建无人机并分配到基站
        for i in range(num_drones):
            pos = random_position(self.size[:2]) + [HEIGHT_DRONE]
            drone = Drone(i, pos, DRONE_SPEED, BATTERY_CAPACITY)
            self.drones.append(drone)
            
            # 分配到覆盖范围内的基站，选择距离最近的基站
            assigned_base = None
            min_distance = float('inf')
            for base in self.base_stations:
                if base.is_in_coverage(pos):
                    distance = calculate_distance(base.position, pos)
                    if distance < min_distance:
                        min_distance = distance
                        assigned_base = base
            
            if assigned_base:
                assigned_base.drones.append(drone)
                drone.base_station = assigned_base
            else:
                print(f"无人机 {i} 不在任何基站覆盖范围内")
        
        for i in range(num_stations):
            pos = random_position(self.size[:2]) + [HEIGHT_POWER_STATION]
            station = PowerStation(i, pos)
            self.power_stations.append(station)

    def update(self):
        """
        更新环境状态，例如无人机位置和任务状态。
        """
        # 为没有任务的无人机分配最近的电站
        for drone in self.drones:
            if not drone.task:
                nearest_station = None
                min_distance = float('inf')
                for station in self.power_stations:
                    # 假设电站有 inspection_needed 属性，表示是否需要巡检
                    if hasattr(station, 'inspection_needed') and station.inspection_needed:
                        distance = calculate_distance(drone.position, station.position)
                        if distance < min_distance:
                            min_distance = distance
                            nearest_station = station
                
                if nearest_station:
                    drone.task = Task(nearest_station.station_id, nearest_station)
                    drone.task.update_status("assigned")
                    print(f"无人机 {drone.drone_id} 分配到电站 {nearest_station.station_id}")
                    nearest_station.inspection_needed = False  # 标记为已分配
        
        # 更新无人机状态
        for drone in self.drones:
            if drone.task and drone.task.status == "assigned":
                drone.move_to(drone.task.target.position)
                data = drone.inspect(drone.task.target)
                if data:
                    drone.send_data(drone.base_station, data)
                    drone.task.update_status("completed")
                    drone.task.target.inspection_needed = True  # 标记为需要巡检
                    drone.task = None