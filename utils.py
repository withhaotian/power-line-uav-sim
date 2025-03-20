"""
7. Utils.py - 工具函数
功能
提供通用的辅助函数。

内容
calculate_distance(pos1, pos2)：计算两点间欧氏距离
random_position(size)：生成随机三维坐标
"""

import math
import random
import numpy as np
from config import *

random.seed(2025)

def calculate_distance(pos1, pos2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(pos1[:2], pos2[:2])))

def random_position(size, bound = 0):
    return [random.uniform(0 + bound, s - bound) for s in size]

def plan_drone_path(drone, power_stations):
    current_pos = drone.position.copy()
    unvisited = [station.position.copy() for station in power_stations]
    path = [current_pos]

    while unvisited:
        nearest_station = min(unvisited, key=lambda pos: np.linalg.norm(np.array(pos) - np.array(current_pos)))
        path.append(nearest_station)
        unvisited.remove(nearest_station)
        current_pos = nearest_station

    drone.position_history = path  # 更新无人机路径

def generate_polygon_positions(center, radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    positions = []
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        positions.append([x, y, HEIGHT_POWER_STATION])  # 固定高度为110米
    return positions

def generate_ring_positions(center, radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    return [[center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle), HEIGHT_POWER_STATION] for angle in angles]

def is_valid_pos(pos, power_stations, base_stations):
    """
        当前pos与其他任意电力塔的距离要超过800米
        并且与基站的距离不小于100米
    """

    for i in range(len(power_stations)):
        if calculate_distance(pos, power_stations[i].position) < 800:
            return False
    for i in range(len(base_stations)):
        if calculate_distance(pos, base_stations[i].position) < 100:
            return False
    return True

class UnionFind:
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, x):
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]

    def union(self, x, y):
        px, py = self.find(x), self.find(y)
        if px == py:
            return
        if self.rank[px] < self.rank[py]:
            px, py = py, px
        self.parent[py] = px
        if self.rank[px] == self.rank[py]:
            self.rank[px] += 1

def generate_mst(positions):
    n = len(positions)
    edges = []
    for i in range(n):
        for j in range(i + 1, n):
            dist = np.linalg.norm(np.array(positions[i]) - np.array(positions[j]))
            edges.append((dist, i, j))
    edges.sort()  # 按距离排序

    uf = UnionFind(n)
    mst_edges = []
    for dist, u, v in edges:
        if uf.find(u) != uf.find(v):
            uf.union(u, v)
            mst_edges.append((u, v))
    return mst_edges