"""
Config.py - 配置文件
功能
存储仿真参数，便于调整。
"""

from param import get_args

args = get_args()

METHOD = args.method  # 算法选择

NUM_DRONES = args.num_drones  # 无人机数量
NUM_BASE_STATIONS = 9  # 基站数量
NUM_POWER_STATIONS = 20  # 电站数量

SPACE_SIZE = [4000, 4000, 100]  # 三维空间尺寸 m
DRONE_SPEED = 5.0  # 无人机速度（米/秒）
BATTERY_CAPACITY = 100.0  # 初始电量

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
MAX_LOAD = 10.0    # 基站最大负载（单位资源）
MAX_ITERATIONS = 200  # 联合优化最大迭代次数
CONVERGENCE_THRESHOLD = 0.005  # 收敛阈值
DISTANCE_EFFICIENT = 0.005   # 飞行距离能效系数

DATA_THRES = 100   # 数据阈值（单位字节）
DATA_RATE = 0.3      # 数据请求速率
TRANS_DEMAND = args.resource   # 传输需求（单位资源）
COMP_DEMAND = args.resource     # 计算需求（单位资源）

PRICE_COMP_BOUND = [0.1, 10]  # 计算资源价格范围（单位1e-2元/单位资源）
PRICE_TRANS_BOUND = [0.8, 10]  # 传输资源价格范围（单位1e-2元/单位资源）

ETA = [0.15, 0.3]   # 价格调整梯度参数
KSI = [0.7, 0.9]   # 调节边际收益反馈力度
MU = [500, 1000.0]     # 基站负载惩罚系数