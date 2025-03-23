"""
Utils.py - 工具函数
功能
提供通用的辅助函数。
"""

import math
import random
import numpy as np
from collections import defaultdict, deque
from heapq import heappush, heappop
import heapq    
from config import *

# random.seed(2025)

def calculate_2d_distance(pos1, pos2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(pos1[:2], pos2[:2])))

def calculate_3d_distance(pos1, pos2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(pos1, pos2)))

def random_position(size, bound = 0):
    return [random.randint(0 + bound, s - bound) for s in size]

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
        当前电力塔pos与其他任意电力塔的距离要超过800米
        并且与基站的距离不小于100米
    """

    for i in range(len(power_stations)):
        if calculate_2d_distance(pos, power_stations[i].position) < 600:
            return False
    for i in range(len(base_stations)):
        if calculate_2d_distance(pos, base_stations[i].position) < 100:
            return False
    for i in range(len(DRONE_SITES)):
        if calculate_2d_distance(pos, DRONE_SITES[i]) < 100:
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

def weighted_kmeans(features, k, weights=[1, 1, 1, 0.5]):  # 权重示例，长度权重影响较小
    centroids = random.sample(features, k)  # 随机初始化中心
    for _ in range(100):  # 迭代次数
        clusters = [[] for _ in range(k)]
        # 分配点到最近的中心
        for feature in features:
            weighted_dists = [sum(w * (f - c) ** 2 for w, f, c in zip(weights, feature, centroid)) 
                              for centroid in centroids]
            cluster_idx = weighted_dists.index(min(weighted_dists))
            clusters[cluster_idx].append(feature)
        # 更新中心
        new_centroids = []
        for cluster in clusters:
            if cluster:
                new_centroid = [sum(f[i] for f in cluster) / len(cluster) for i in range(4)]
                new_centroids.append(new_centroid)
            else:
                new_centroids.append(random.choice(features))  # 空簇随机重置
        if new_centroids == centroids:
            break
        centroids = new_centroids
    return clusters

def extract_features(mst_edges, power_stations):
    features = []
    for u, v in mst_edges:
        pos1, pos2 = power_stations[u].position, power_stations[v].position
        x_e = (pos1[0] + pos2[0]) / 2
        y_e = (pos1[1] + pos2[1]) / 2
        z_e = (pos1[2] + pos2[2]) / 2
        l_e = calculate_3d_distance(pos1, pos2)
        features.append([u, v, x_e, y_e, z_e, l_e])
    return features

def find_path(start, end, mst_edges):
    """使用 BFS 在 MST 中找到两节点间的路径"""
    graph = defaultdict(list)
    for edge_idx, (a, b) in enumerate(mst_edges):
        graph[a].append((b, edge_idx))
        graph[b].append((a, edge_idx))
    
    queue = deque([(start, [])])
    visited = set()
    while queue:
        current, path = queue.popleft()
        if current == end:
            return [edge for _, edge in path]
        visited.add(current)
        for neighbor, edge_idx in graph[current]:
            if neighbor not in visited:
                queue.append((neighbor, path + [(current, edge_idx)]))
    return []

def connected_kmeans(mst_edges, power_stations, num_clusters):
    """改进的 K-means 聚类，确保 MST 边分配到连通子图且大小均衡"""
    n = len(power_stations)
    positions = [station.position for station in power_stations]
    
    # 构建边的邻接表 (节点 -> 边索引)
    edge_graph = defaultdict(list)
    for edge_idx, (u, v) in enumerate(mst_edges):
        edge_graph[u].append(edge_idx)
        edge_graph[v].append(edge_idx)
    
    # 选择空间上分散的种子边
    seed_edges = []
    available_edges = set(range(len(mst_edges)))
    for _ in range(num_clusters):
        if not seed_edges:
            seed_edge = random.choice(list(available_edges))
        else:
            max_min_dist = -1
            best_edge = None
            for edge_idx in available_edges:
                mid = [(positions[mst_edges[edge_idx][0]][0] + positions[mst_edges[edge_idx][1]][0]) / 2,
                       (positions[mst_edges[edge_idx][0]][1] + positions[mst_edges[edge_idx][1]][1]) / 2]
                min_dist = min(calculate_2d_distance(
                    mid,
                    [(positions[mst_edges[se][0]][0] + positions[mst_edges[se][1]][0]) / 2,
                     (positions[mst_edges[se][0]][1] + positions[mst_edges[se][1]][1]) / 2]
                ) for se in seed_edges)
                if min_dist > max_min_dist:
                    max_min_dist = min_dist
                    best_edge = edge_idx
            seed_edge = best_edge
        seed_edges.append(seed_edge)
        available_edges.remove(seed_edge)
    
# Initialize clusters and assigned edges
    clusters = [set([edge_idx]) for edge_idx in seed_edges]
    assigned_edges = set(seed_edges)
    
    # Build edge connectivity (edges sharing nodes)
    edge_connectivity = defaultdict(list)
    for i, (u1, v1) in enumerate(mst_edges):
        for j, (u2, v2) in enumerate(mst_edges):
            if i != j and (u1 == u2 or u1 == v2 or v1 == u2 or v1 == v2):
                edge_connectivity[i].append(j)
    
    # Initialize priority queue with cluster sizes
    cluster_sizes = [len(c) for c in clusters]
    pq = [(size, idx) for idx, size in enumerate(cluster_sizes)]
    heapq.heapify(pq)  # Convert list to heap
    
    # Expand clusters using the priority queue
    while len(assigned_edges) < len(mst_edges):
        if not pq:  # If queue is empty, stop
            break
        _, cluster_idx = heapq.heappop(pq)  # Get smallest cluster
        if not clusters[cluster_idx]:  # Skip empty clusters
            continue
        queue = deque(clusters[cluster_idx])  # BFS queue for edges in cluster
        expanded = False
        while queue and not expanded:
            current_edge_idx = queue.popleft()
            for neighbor_edge_idx in edge_connectivity[current_edge_idx]:
                if neighbor_edge_idx not in assigned_edges:
                    clusters[cluster_idx].add(neighbor_edge_idx)
                    assigned_edges.add(neighbor_edge_idx)
                    queue.append(neighbor_edge_idx)
                    expanded = True
                    break
            if expanded:
                break
        if expanded:
            cluster_sizes[cluster_idx] += 1  # Update size
            heapq.heappush(pq, (cluster_sizes[cluster_idx], cluster_idx))  # Push updated tuple
    
    # 处理剩余未分配的边
    remaining_edges = set(range(len(mst_edges))) - assigned_edges
    while remaining_edges:
        edge_idx = remaining_edges.pop()
        u, v = mst_edges[edge_idx]
        edge_mid = [(positions[u][0] + positions[v][0]) / 2, 
                    (positions[u][1] + positions[v][1]) / 2]
        
        # 找到距离最近的簇
        min_dist = float('inf')
        best_cluster_idx = None
        for i, cluster in enumerate(clusters):
            for existing_edge_idx in cluster:
                eu, ev = mst_edges[existing_edge_idx]
                cluster_mid = [(positions[eu][0] + positions[ev][0]) / 2, 
                              (positions[eu][1] + positions[ev][1]) / 2]
                dist = calculate_2d_distance(edge_mid, cluster_mid)
                if dist < min_dist:
                    min_dist = dist
                    best_cluster_idx = i
        
        if best_cluster_idx is not None:
            clusters[best_cluster_idx].add(edge_idx)
            assigned_edges.add(edge_idx)
            
            # 确保簇内连通性
            uf = UnionFind(n)
            for e in clusters[best_cluster_idx]:
                u_node, v_node = mst_edges[e]
                uf.union(u_node, v_node)
            if uf.find(u) != uf.find(v):
                path = find_path(u, v, mst_edges)
                for path_edge in path:
                    if path_edge not in assigned_edges:
                        clusters[best_cluster_idx].add(path_edge)
                        assigned_edges.add(path_edge)
    
    # 转换为特征格式
    feature_clusters = []
    for cluster in clusters:
        cluster_edges = [[positions[mst_edges[i][0]], positions[mst_edges[i][1]]] for i in cluster]
        feature_clusters.append(cluster_edges)
    
    # 验证所有边都被覆盖
    assert len(assigned_edges) == len(mst_edges), f"未覆盖所有边: {len(assigned_edges)}/{len(mst_edges)}"
    return feature_clusters