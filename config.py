"""
8. Config.py - 配置文件
功能
存储仿真参数，便于调整。
"""

SPACE_SIZE = [4000, 4000, 150]  # 三维空间尺寸 m
DRONE_SPEED = 5.0  # 无人机速度（米/秒）
BATTERY_CAPACITY = 100.0  # 初始电量
SIMULATION_STEPS = 50  # 仿真步数

HEIGHT_BASE_STATION = 10  # 基站高度（米）
HEIGHT_POWER_STATION = 100  # 电站高度（米）
HEIGHT_DRONE = 110  # 无人机飞行高度（米）

COVERAGE_RADIUS = 600  # 覆盖半径（米）

# 定义四个固定无人机站点
DRONE_SITES = [
    [500, 2000, 1],
    [3500, 2000, 1],
    [2000, 500, 1],
    [2000, 3500, 1]
]