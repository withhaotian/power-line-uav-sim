"""
6. Simulation.py - 仿真主程序
功能
运行整个仿真过程，控制时间步进。

内容
初始化环境和实体
模拟时间步进，更新状态
调用 Logger 记录数据
"""

from environment import Environment
from config import SIMULATION_STEPS, SPACE_SIZE
from logger import Logger
import datetime

def run_simulation():
    env = Environment(SPACE_SIZE)
    env.initialize(num_drones=3, num_stations=5)
    logger = Logger()

    for step in range(SIMULATION_STEPS):
        env.update()
        logger.log(step, env.drones, env.power_stations)
    logger.save("logs/log_{}.txt".format(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")))

if __name__ == "__main__":
    run_simulation()