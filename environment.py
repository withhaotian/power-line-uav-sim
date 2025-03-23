"""
Environment.py - 三维空间环境类
功能
模拟三维空间，管理所有实体之间的交互。
"""
import numpy as np
from base_station import BaseStation
from drone import Drone
from power_station import PowerStation
from utils import *
from config import *
from collections import defaultdict
import heapq

class Environment:
    def __init__(self, size, logger):
        """
        初始化三维空间环境。
        
        参数:
            size: 空间尺寸，例如 [x_max, y_max, z_max]
        """
        self.size = size  # [x_max, y_max, z_max]
        self.drones = []  # 无人机列表
        self.power_stations = []  # 电站列表
        self.base_stations = []  # 地面基站列表

        self.clusters = []  # 聚类结果
        self.features = []  # 用于 k-means 聚类的数据特征
        self.mst_edges = []  # 电线的最小生成树边集合

        self.obstacles = []  # 障碍物集合

        self.logger = logger

    def initialize(self, num_drones, num_stations, num_base_stations):
        """
        初始化环境，创建基站、无人机和电站，并进行分配。
        
        参数:
            num_drones: 无人机数量
            num_stations: 电站数量
            num_base_stations: 基站数量
        """
        
        ''' 网络拓扑预处理'''
        center = [self.size[0] / 2, self.size[1] / 2]

        # 基站分布
        self.base_stations = []
        if num_base_stations == 1:
            pos = [center[0], center[1], HEIGHT_BASE_STATION]
            self.base_stations.append(BaseStation(0, pos, COVERAGE_RADIUS))
        elif num_base_stations == 2:
            for i in range(2):
                x = center[0] + (i - 0.5) * (self.size[0] / 4)
                pos = [x, center[1], HEIGHT_BASE_STATION]
                self.base_stations.append(BaseStation(i, pos, COVERAGE_RADIUS))
        elif num_base_stations == 3:
            radius = self.size[0] / 6
            angles = np.linspace(0, 2 * np.pi, 4, endpoint=False)[:3]  # 三角形
            for i, angle in enumerate(angles):
                x = center[0] + radius * np.cos(angle)
                y = center[1] + radius * np.sin(angle)
                pos = [x, y, HEIGHT_BASE_STATION]
                self.base_stations.append(BaseStation(i, pos, COVERAGE_RADIUS))
        elif num_base_stations == 4:
            offset = self.size[0] / 6
            for i, (dx, dy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
                pos = [center[0] + dx * offset, center[1] + dy * offset, HEIGHT_BASE_STATION]
                self.base_stations.append(BaseStation(i, pos, COVERAGE_RADIUS))
        elif num_base_stations == 5:
            offset = self.size[0] / 6
            positions = [[center[0] + offset * dx, center[1] + offset * dy, HEIGHT_BASE_STATION] 
                         for dx, dy in [(-1, 1), (0, 1), (1, 1), (-0.5, -1), (0.5, -1)]]
            for i, pos in enumerate(positions):
                self.base_stations.append(BaseStation(i, pos, COVERAGE_RADIUS))
        elif num_base_stations == 6:
            offset = self.size[0] / 6
            positions = [[center[0] + offset * dx, center[1] + offset * dy, HEIGHT_BASE_STATION] 
                         for dx, dy in [(-1, 1), (0, 1), (1, 1), (-1, -1), (0, -1), (1, -1)]]
            for i, pos in enumerate(positions):
                self.base_stations.append(BaseStation(i, pos, COVERAGE_RADIUS))
        else:
            grid_size = int(np.ceil(np.sqrt(num_base_stations)))
            step = self.size[0] / (grid_size + 1)
            for i in range(grid_size):
                for j in range(grid_size):
                    if len(self.base_stations) < num_base_stations:
                        pos = [center[0] + (i - (grid_size - 1) / 2) * step, 
                               center[1] + (j - (grid_size - 1) / 2) * step, HEIGHT_BASE_STATION]
                        self.base_stations.append(BaseStation(len(self.base_stations), pos, COVERAGE_RADIUS))
        
        # 创建无人机从无人机站点出发
        for i in range(num_drones):
            start_pos = DRONE_SITES[i % len(DRONE_SITES)]  # 从 4 个站点中随机选择一个
            drone = Drone(i, start_pos, DRONE_SPEED, BATTERY_CAPACITY)
            drone.position_history = [start_pos]
            self.drones.append(drone)
        
        # 创建电力塔
        for i in range(num_stations):
            pos = random_position(self.size[:2], 100) + [HEIGHT_POWER_STATION]
            while(not is_valid_pos(pos, self.power_stations, self.base_stations)):
                pos = random_position(self.size[:2], 100) + [HEIGHT_POWER_STATION]
            station = PowerStation(i, pos)
            self.power_stations.append(station)
        
        # 电力塔电线连接
        power_station_positions = np.array([station.position for station in self.power_stations])
        self.mst_edges = generate_mst(power_station_positions)
        # print("电线的最小生成树边集合：", self.mst_edges)
        
        ''' k-means 任务分割'''
        # self.features = extract_features(self.mst_edges, self.power_stations)
        # cluster_features = weighted_kmeans(self.features, num_drones)
        # for features in cluster_features:
        #     pos = []
        #     for item in features:
        #         pos1 = self.power_stations[item[0]].position
        #         pos2 = self.power_stations[item[1]].position
        #         pos.append([pos1, pos2])
        #     self.clusters.append(pos)
        # print('特征数据：', self.features)
        # print('聚类结果：', len(self.clusters), '个聚类', self.clusters)
        self.clusters = connected_kmeans(self.mst_edges, self.power_stations, num_drones)
        print("连通聚类结果：")
        print([len(cluster) for cluster in self.clusters], '个电线')
        # for i, cluster in enumerate(self.clusters):
        #     print('簇', i, '：', cluster)

        ''' 初始化路径规划'''
        self.obstacles = {tuple(station.position) for station in self.power_stations}  # 电站作为障碍物
        self.assign_clusters_to_drones()  # 分配簇
        for drone in self.drones:
            cluster = self.clusters[drone.cluster_id]
            path = self.plan_drone_path(drone, cluster)
            drone.position_history = path  # 更新路径历史
            print(f"无人机 {drone.drone_id} 的初始路径轨迹：", path)

    def assign_clusters_to_drones(self):
        """为每个无人机分配距离最近的未认领簇"""
        assigned_clusters = set()  # 已分配的簇索引
        tower_positions = {i: station.position for i, station in enumerate(self.power_stations)}
        
        for drone in self.drones:
            min_dist = float('inf')
            best_cluster_idx = None
            start_pos = drone.position_history[0]  # 无人机的充电站位置
            
            # 遍历所有簇，寻找最近的未分配簇
            for i, cluster in enumerate(self.clusters):
                if i in assigned_clusters:
                    continue
                # 获取簇中的电力塔索引
                tower_indices = set()
                for feature in cluster:
                    for idx, pos in tower_positions.items():
                        if np.allclose(pos[:2], feature[0][:2]) or np.allclose(pos[:2], feature[1][:2]):
                            tower_indices.add(idx)
                # 计算到簇中最近节点的距离
                min_tower_dist = min(
                    calculate_2d_distance(start_pos, tower_positions[idx])
                    for idx in tower_indices
                )
                if min_tower_dist < min_dist:
                    min_dist = min_tower_dist
                    best_cluster_idx = i
            
            if best_cluster_idx is not None:
                drone.cluster_id = best_cluster_idx
                assigned_clusters.add(best_cluster_idx)
            else:
                raise ValueError("没有可分配的簇给无人机！")
    
    def plan_drone_path(self, drone, cluster):
        """为无人机规划遍历簇内所有边的优化路径"""
        # 电力塔位置字典
        tower_positions = {i: station.position for i, station in enumerate(self.power_stations)}
        
        # 获取簇中的电力塔索引
        tower_indices = set()
        for feature in cluster:
            for i, pos in tower_positions.items():
                if np.allclose(pos[:2], feature[0][:2]) or np.allclose(pos[:2], feature[1][:2]):
                    tower_indices.add(i)
        
        # 构建簇内的连接图（带边标识）
        graph = defaultdict(list)  # 节点 -> [(邻居, 边索引)]
        edge_set = set()  # 所有边的集合
        edge_id = 0
        edge_map = {}  # (u, v) 或 (v, u) -> 边索引
        for u, v in self.mst_edges:
            if u in tower_indices and v in tower_indices:
                graph[u].append((v, edge_id))
                graph[v].append((u, edge_id))
                edge_set.add(edge_id)
                edge_map[(u, v)] = edge_id
                edge_map[(v, u)] = edge_id
                edge_id += 1
        
        # 选择度数为1的节点作为出发点
        # start_tower = None
        # for idx in tower_indices:
        #     if len(graph[idx]) == 1:
        #         start_tower = idx
        #         break
        # if start_tower is None:
        #     start_tower = next(iter(tower_indices))  # 如果没有度数为1的节点，选择任意节点
        
        # 选择距离充电站最近的度数为1的节点作为出发点
        start_pos = drone.position_history[0]  # 充电站位置
        degree_one_nodes = [idx for idx in tower_indices if len(graph[idx]) == 1]
        if degree_one_nodes:
            start_tower = min(degree_one_nodes, 
                            key=lambda idx: calculate_2d_distance(start_pos, tower_positions[idx]))
        else:
            start_tower = next(iter(tower_indices))  # 如果没有度数为1的节点，选择任意节点
        
        # 初始化路径，从充电站到起点
        path = [drone.position_history[0]]
        start_path = self.a_star_search(drone.position_history[0], tower_positions[start_tower], graph, tower_positions)
        if start_path:
            path.extend(start_path[1:])  # 跳过起点
        
        # 遍历所有边
        visited_edges = set()  # 已访问的边索引
        
        def traverse_edges(current):
            """基于边的 DFS，确保覆盖所有边"""
            # 检查当前节点的所有邻边
            for neighbor, edge_id in graph[current]:
                if edge_id not in visited_edges:
                    visited_edges.add(edge_id)
                    # 使用 A* 计算到邻居的最优路径
                    a_star_path = self.a_star_search(path[-1], tower_positions[neighbor], graph, tower_positions)
                    if a_star_path:
                        path.extend(a_star_path[1:])  # 跳过起点
                    traverse_edges(neighbor)
        
        traverse_edges(start_tower)
        
        # 检查并补充遗漏的边
        while visited_edges != edge_set:
            remaining_edges = edge_set - visited_edges
            edge_id = next(iter(remaining_edges))  # 取一条未访问的边
            for (u, v), eid in edge_map.items():
                if eid == edge_id:
                    start_node, end_node = u, v
                    break
            # 从当前位置到未访问边的起点
            if path[-1] != tower_positions[start_node]:
                a_star_path = self.a_star_search(path[-1], tower_positions[start_node], graph, tower_positions)
                if a_star_path:
                    path.extend(a_star_path[1:])
            # 访问边
            visited_edges.add(edge_id)
            a_star_path = self.a_star_search(tower_positions[start_node], tower_positions[end_node], graph, tower_positions)
            if a_star_path:
                path.extend(a_star_path[1:])
            # 从新位置继续遍历
            traverse_edges(end_node)
        
        # 选择最近的充电站
        last_pos = path[-1]
        min_dist = float('inf')
        nearest_station = None
        for station in DRONE_SITES:  # 假设 charging_stations 已定义
            dist = calculate_2d_distance(last_pos, station)
            if dist < min_dist:
                min_dist = dist
                nearest_station = station
        
        # 使用 A* 规划到最近充电站的路径
        if nearest_station:
            a_star_path = self.a_star_search(last_pos, nearest_station, graph, tower_positions)
            if a_star_path:
                path.extend(a_star_path[1:])  # 跳过起点
        
        return path

    def a_star_search(self, start_pos, goal_pos, graph, positions):
        """A* 算法寻找从起点位置到目标位置的最优路径"""
        def heuristic(pos1, pos2):
            return calculate_2d_distance(pos1, pos2)
        
        # 将位置映射回节点索引
        start = None
        goal = None
        for idx, pos in positions.items():
            if np.allclose(start_pos[:2], pos[:2]):
                start = idx
            if np.allclose(goal_pos[:2], pos[:2]):
                goal = idx
        
        # 如果起点或终点不在图中，直接返回直线路径
        if start is None or goal is None:
            return [start_pos, goal_pos]
        
        open_set = [(0, start)]  # 优先队列 (f_score, 节点)
        came_from = {}  # 前驱节点
        g_score = {start: 0}  # 起点到当前节点的实际成本
        f_score = {start: heuristic(positions[start], goal_pos)}  # 估计总成本
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal:
                # 重建路径
                path = []
                while current in came_from:
                    path.append(positions[current])
                    current = came_from[current]
                path.append(positions[start])
                return path[::-1]  # 逆序返回路径
            
            for neighbor, _ in graph[current]:  # 忽略边索引，仅使用邻居节点
                tentative_g_score = g_score[current] + calculate_2d_distance(positions[current], positions[neighbor])
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(positions[neighbor], goal_pos)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # 如果无法到达目标，返回直线路径
        return [start_pos, goal_pos]

    def run(self):
        """
        更新环境状态，例如无人机位置和任务状态。
        """
        pass