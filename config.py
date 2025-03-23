"""
Config.py - 配置文件
功能
存储仿真参数，便于调整。
"""

SPACE_SIZE = [4000, 4000, 100]  # 三维空间尺寸 m
DRONE_SPEED = 5.0  # 无人机速度（米/秒）
BATTERY_CAPACITY = 100.0  # 初始电量
SIMULATION_STEPS = 50  # 仿真步数

HEIGHT_BASE_STATION = 10  # 基站高度（米）
HEIGHT_POWER_STATION = 50  # 电站高度（米）
HEIGHT_DRONE = 55  # 无人机飞行高度（米）

COVERAGE_RADIUS = 600  # 覆盖半径（米）

# 定义四个固定无人机站点
DRONE_SITES = [
    [500, 2000, 1],
    [3500, 2000, 1],
    [2000, 500, 1],
    [2000, 3500, 1]
]

ALPHA = 0.1  # 水平距离能耗系数
BETA = 0.2   # 垂直距离能耗系数
MU = 1.0     # 基站负载惩罚系数
MAX_LOAD = 5    # 基站最大负载（单位资源）
P_TRANS = 0.5  # 传输价格（单位资源）
P_COMP = 0.8   # 计算价格（单位资源）
MAX_ITERATIONS = 100  # 联合优化最大迭代次数
CONVERGENCE_THRESHOLD = 0.01  # 收敛阈值