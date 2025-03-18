"""
1. Drone.py - 无人机类
功能
定义无人机的属性和行为，模拟无人机在巡检任务中的操作。

内容
属性：
    battery：电量（初始值可配置，每次移动或巡检消耗电量）
    position：当前位置（三维坐标，例如 [x, y, z]）
    speed：移动速度（单位：米/秒）
    task：当前任务（Task 对象或 None）
方法：
    request_task(base_station)：向基站请求任务
    move_to(target_position)：移动到目标位置，更新位置并消耗电量
    inspect(power_station)：执行巡检任务，模拟采集数据
    send_data(base_station)：将巡检数据发送到基站
    update_battery()：根据移动或巡检更新电量
"""

from utils import calculate_distance

class Drone:
    def __init__(self, drone_id, position, speed, battery_capacity):
        self.drone_id = drone_id
        self.position = position  # [x, y, z]
        self.speed = speed
        self.battery = battery_capacity
        self.task = None

    def request_task(self, base_station):
        self.task = base_station.assign_task(self)
        return self.task

    def move_to(self, target_position):
        distance = calculate_distance(self.position, target_position)
        time = distance / self.speed
        self.battery -= time * 0.1  # 假设每秒耗电0.1单位
        self.position = target_position
        return distance

    def inspect(self, power_station):
        if self.position == power_station.position:
            data = f"Video data from {power_station.station_id}"
            self.battery -= 5  # 巡检固定耗电
            return data
        return None

    def send_data(self, base_station, data):
        base_station.receive_data(self.drone_id, data)