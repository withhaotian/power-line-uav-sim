import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import numpy as np
from environment import Environment
from config import SIMULATION_STEPS, SPACE_SIZE
from utils import *

# 设置全局字体为 Helvetica，并将默认字体大小设为 26
plt.rcParams['font.family'] = 'Helvetica'

def visualize_2d(env, save_fig = False):
    fig, ax = plt.subplots(figsize=(10, 8))

    # 绘制电站位置
    station_positions = np.array([station.position for station in env.power_stations])
    for i, station in enumerate(env.power_stations):
        ax.scatter(station.position[0], station.position[1], c='b', marker='^', s=100, edgecolors='black',
                   label='Power Tower' if 'Power Tower' not in ax.get_legend_handles_labels()[1] else "", zorder=10)
        # ax.text(station.position[0], station.position[1], f'PS {station.station_id}', color='blue')

    # 绘制 MST 电线连接
    # colors = list(iter(plt.cm.viridis(np.linspace(0, 1, len(env.drones)))))
    # mst_edges = generate_mst(station_positions)
    # for i, cluster in enumerate(env.clusters):
    #     def is_target_in_clusters(target, clusters):
    #         for i, cluster in enumerate(clusters):
    #             if (target[0] == cluster[0] and target[1] == cluster[1]) or (target[0] == cluster[1] and target[1] == cluster[0]):
    #                 return True  # 存在于第 i 个簇中
    #         return False  # 未找到
    #     for u, v in mst_edges:
    #         is_this_cluster = is_target_in_clusters([list(station_positions[u]), list(station_positions[v])], cluster)
    #         if is_this_cluster:
    #             ax.plot([station_positions[u][0], station_positions[v][0]],
    #                     [station_positions[u][1], station_positions[v][1]], 
    #                     color=colors[i], linestyle='-', linewidth=1.5, alpha=0.7, zorder=15)
    
    # 绘制无人机路径
    colors = iter(plt.cm.rainbow(np.linspace(0, 1, len(env.drones))))
    for drone in env.drones:
        path = drone.position_history
        color = next(colors)
        ax.plot([p[0] for p in path], [p[1] for p in path], color=color, linestyle='--', linewidth=2, alpha=0.9, zorder=10)

        arrow_step = max(len(path) // 20, 1)  # 控制箭头密度
        for i in range(0, len(path) - 1, arrow_step):
            x = path[i][0]
            y = path[i][1]
            dx = path[i + 1][0] - x
            dy = path[i + 1][1] - y
            ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color=color, width=0.005, headwidth=4, headlength=6)

    # 绘制基站位置及其覆盖范围圆圈
    for base in env.base_stations:
        # 基站标记
        ax.scatter(base.position[0], base.position[1], c='red', marker='s', s=100, edgecolors='black', linewidth=1.5,
                   label='Base Station' if 'Base Station' not in ax.get_legend_handles_labels()[1] else "", zorder=10)
        # ax.text(base.position[0], base.position[1], f'BS {base.base_id}', color='red', fontsize=26, fontweight='bold')
        
        # 覆盖范围圆圈
        circle = Circle((base.position[0], base.position[1]), COVERAGE_RADIUS, 
                        edgecolor='black', facecolor='none', linestyle='--', linewidth=1, alpha=0.5)
        ax.add_patch(circle)
    
    # 绘制固定无人机站点
    for site in DRONE_SITES:
        ax.scatter(site[0], site[1], c='green', marker='*', s=200, edgecolors='black', linewidth=1.5,
                   label='UAV Charging Station' if 'UAV Charging Station' not in ax.get_legend_handles_labels()[1] else "", zorder=10)
    
    # 设置坐标轴标签和刻度
    ax.set_xlabel('X (m)', fontsize=20, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=20, fontweight='bold')
    ax.set_xlim(0, SPACE_SIZE[0])
    ax.set_ylim(0, SPACE_SIZE[1])
    ax.tick_params(axis='both', which='major', labelsize=20)  # 设置刻度大小为 26
    ax.set_aspect('equal', adjustable='box')
    plt.legend(fontsize=20, ncol=2, bbox_to_anchor=(0.46, 1.2),loc='upper center', frameon=True)
    plt.tight_layout()
    if save_fig:
        plt.savefig('2d_sim_env.pdf', dpi=300, bbox_inches='tight')
    plt.show()

def visualize_3d(env, save_fig = False):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制地面基站（柱状图）
    base_station_positions = np.array([station.position for station in env.base_stations])
    for i, station in enumerate(env.base_stations):
        x, y, z = station.position[0], station.position[1], 0
        dx, dy, dz = 40, 40, HEIGHT_BASE_STATION
        ax.bar3d(x, y, z, dx, dy, dz, color='r', label='Base Station' if 'Base Station' not in ax.get_legend_handles_labels()[1] else "", alpha=0.7, zorder=10)
        # ax.text(x, y, station.position[2], f'BS {station.base_id}', color='blue')

    # 绘制电站（柱状图）
    station_positions = np.array([station.position for station in env.power_stations])
    for i, station in enumerate(env.power_stations):
        x, y, z = station.position[0], station.position[1], 0
        dx, dy, dz = 40, 40, station.position[2]
        ax.bar3d(x, y, z, dx, dy, dz, color='b', label='Power Tower' if 'Power Tower' not in ax.get_legend_handles_labels()[1] else "", alpha=0.7, zorder=10)
        # ax.text(x, y, station.position[2], f'PS {station.station_id}', color='blue')

    # 绘制 MST 电线连接
    mst_edges = generate_mst(station_positions)
    for u, v in mst_edges:
        ax.plot([station_positions[u][0], station_positions[v][0]],
                [station_positions[u][1], station_positions[v][1]],
                [station_positions[u][2], station_positions[v][2]], 
                color='gray', linestyle='-', linewidth=1.5, alpha=0.7, zorder=5)    
    
    # 绘制固定无人机站点
    for site in DRONE_SITES:
        ax.scatter(site[0], site[1], site[2], c='green', marker='*', s=200, edgecolors='black', linewidth=1.5,
                   label='UAV Charging Station' if 'UAV Charging Station' not in ax.get_legend_handles_labels()[1] else "", zorder=10)

    # 设置坐标轴标签和刻度
    ax.set_xlabel('X (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=14, fontweight='bold')
    ax.set_xlim(0, SPACE_SIZE[0])
    ax.set_ylim(0, SPACE_SIZE[1])
    ax.set_zlim(0, SPACE_SIZE[2])
    ax.tick_params(axis='both', which='major', labelsize=14)  # 设置刻度大小为 26
    ax.legend(fontsize=18)  # 设置图例字体大小为 26
    
    plt.tight_layout()
    if save_fig:
        plt.savefig('3d_sim_env.pdf', dpi=300)
    plt.show()

def run_simulation():
    env = Environment(SPACE_SIZE)
    env.initialize(num_drones=5, num_stations=20, num_base_stations=9)  # 初始化环境

    # env.update()  # 更新仿真状态
    visualize_3d(env)
    visualize_2d(env)

if __name__ == "__main__":
    run_simulation()