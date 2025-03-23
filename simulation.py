"""
Simulation.py - 仿真主程序
功能
运行整个仿真过程，控制时间步进。
"""

from environment import Environment
from config import SIMULATION_STEPS, SPACE_SIZE
from logger import Logger
import datetime
from visual import visualize_2d, visualize_3d

def run_simulation():
    logger = Logger()
    env = Environment(SPACE_SIZE, logger)
    env.initialize(num_drones=4, num_stations=20, num_base_stations=9)

    visualize_2d(env)

    # env.run()
    # logger.log(env.drones, env.power_stations, env.base_stations)
    # logger.save("logs/log_{}.txt".format(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")))

if __name__ == "__main__":
    run_simulation()