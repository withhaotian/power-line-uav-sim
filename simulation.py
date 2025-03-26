"""
Simulation.py - 仿真主程序
功能
运行整个仿真过程，控制时间步进。
"""

from environment import Environment
from config import SPACE_SIZE
from logger import Logger
import datetime
from visual import visualize_2d, visualize_3d
from utils import *

def run_simulation():
    logger = Logger()
    env = Environment(SPACE_SIZE, logger)
    env.initialize(num_drones=NUM_DRONES, num_stations=NUM_POWER_STATIONS, num_base_stations=NUM_BASE_STATIONS)
    env.run()

    # visualize_2d(env)

    # logger.log(env.drones, env.power_stations, env.base_stations)
    # logger.save("logs/log_{}.txt".format(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")))

if __name__ == "__main__":
    run_simulation()