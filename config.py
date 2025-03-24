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

ALPHA = 10.0  # 水平距离能耗系数
BETA = 2.0   # 垂直距离能耗系数
MU = 50.0     # 基站负载惩罚系数
MAX_LOAD = 3.0    # 基站最大负载（单位资源）
MAX_ITERATIONS = 200  # 联合优化最大迭代次数
CONVERGENCE_THRESHOLD = 0.01  # 收敛阈值

DATA_THRES = 100   # 数据阈值（单位字节）
DATA_RATE = 0.8      # 数据请求速率
TRANS_DEMAND = DATA_THRES*8   # 传输需求（单位资源）
COMP_DEMAND = 100     # 计算需求（单位资源）

PRICE_COMP_BOUND = [0.5, 50]  # 计算资源价格范围（单位元/单位资源）
PRICE_TRANS_BOUND = [1, 100]  # 传输资源价格范围（单位元/单位资源）

ETA = 0.4   # 价格调整梯度参数
KSI = 0.8   # 调节边际收益反馈力度