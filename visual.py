import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 可视化函数
def visualize(env, step):
    # 创建三维图形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制无人机
    for drone in env.drones:
        ax.scatter(drone.position[0], drone.position[1], drone.position[2], c='r', marker='o')
        ax.text(drone.position[0], drone.position[1], drone.position[2], f'Drone {drone.drone_id}', color='red')

    # 绘制电站
    for station in env.power_stations:
        ax.scatter(station.position[0], station.position[1], station.position[2], c='b', marker='^')
        ax.text(station.position[0], station.position[1], station.position[2], f'Station {station.station_id}', color='blue')

    # 绘制基站
    ax.scatter(env.base_station.position[0], env.base_station.position[1], env.base_station.position[2], c='g', marker='s')
    ax.text(env.base_station.position[0], env.base_station.position[1], env.base_station.position[2], 'Base Station', color='green')

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Simulation Step {step}')
    
    # 显示图形
    plt.show()

# 集成到仿真中
from environment import Environment
from config import SIMULATION_STEPS, SPACE_SIZE

def run_simulation():
    env = Environment(SPACE_SIZE)
    env.initialize(num_drones=5, num_stations=10)  # 初始化环境

    for step in range(SIMULATION_STEPS):
        env.update()  # 更新仿真状态
        visualize(env, step)  # 可视化当前步骤

if __name__ == "__main__":
    run_simulation()