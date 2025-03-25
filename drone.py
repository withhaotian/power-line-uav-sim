"""
Drone.py - 无人机类
功能
定义无人机的属性和行为，模拟无人机在巡检任务中的操作。
"""

from utils import *
from config import *

class Drone:
    def __init__(self, drone_id, position, speed, battery_capacity):
        self.drone_id = drone_id
        self.position = position  # [x, y, z]
        self.speed = speed
        self.battery = battery_capacity
        self.base_station = None
        self.position_history = None
        self.cluster_id = None
        self.trans_demand = TRANS_DEMAND   # 传输需求
        self.comp_demand = COMP_DEMAND   # 计算需求  
        self.data_thres = DATA_THRES   # 数据阈值

        self.request_history = []  # 历史请求记录   [trans_demand, comp_demand, best_base_station_id]

    def move_to(self, target_position):
        # 计算水平距离 d_xy 和高度变化 |Δz|
        d_xy = calculate_2d_distance(self.position, target_position)  # 现有函数计算二维距离
        delta_z = abs(target_position[2] - self.position[2])  # z 轴高度差
        energy_consumed = ALPHA * d_xy + BETA * delta_z  # 根据公式计算能耗
        self.battery -= energy_consumed  # 更新电量
        self.position = target_position
        return d_xy

    def inspect(self, power_station):
        if self.position == power_station.position:
            data = f"来自电站 {power_station.station_id} 的4K视频数据"
            self.battery -= 5  # 巡检固定耗电
            return data
        return None

    def send_data(self, base_station, data):
        base_station.receive_data(self.drone_id, data)
    
    def calculate_cost(self, base_station, start_pos_idx, end_pos_idx, curr_drone_pos, new_drone_pos=None):
        """计算无人机成本"""
        if not self.position_history or len(self.position_history) < 2:
            return 0
        cost = 0
        service_cost = 0
        energy_cost = 0
        extra_cost = 0
        # 服务成本
        service_cost += (base_station.price_trans * self.trans_demand + base_station.price_comp * self.comp_demand)
        # 能耗成本
        # if self.position_history:
        #     i = start_pos_idx
        #     while i < end_pos_idx:
        #         v_a, v_b = self.position_history[i], self.position_history[i + 1]
        #         d_ab = calculate_2d_distance(v_a, v_b)
        #         # print('飞行距离：', d_ab)
        #         z_diff = abs(v_a[2] - v_b[2])
        #         energy_cost += ALPHA * d_ab + BETA * z_diff
        #         i += 1
        # 额外飞行至覆盖范围内的基站的能耗成本
        if new_drone_pos:
            d_ab = calculate_2d_distance(curr_drone_pos, new_drone_pos)
            d_ab *= 2  # 往返飞行
            z_diff = abs(curr_drone_pos[2] - new_drone_pos[2])
            extra_cost += ALPHA * d_ab + BETA * z_diff
        # print('额外能耗成本：', extra_cost)
        cost = service_cost + energy_cost + extra_cost

        # 记录所有历史数据请求，为基站统计收益
        self.request_history.append([self.trans_demand, self.comp_demand, base_station.base_id])
        
        return cost

    def select_best_base(self, curr_pos, start_pos_idx, end_pos_idx, base_stations):
        """最优卸载决策"""
        min_cost = float('inf')
        best_bs = None
        best_new_pos = None
        if METHOD == 'ours':
            for bs in base_stations:
                if bs.is_in_coverage(curr_pos):
                    cost = self.calculate_cost(bs, start_pos_idx, end_pos_idx, curr_pos)
                    if cost < min_cost:
                        min_cost = cost
                        best_bs = bs
                        best_new_pos = None
                else:
                    # 若当前基站不在覆盖范围内，则尝试将无人机移动到刚好进入覆盖范围时的三维坐标
                    new_pos = self.get_minimal_move_point(curr_pos, bs.position)
                    cost = self.calculate_cost(bs, start_pos_idx, end_pos_idx, curr_pos, new_pos)
                    if cost < min_cost:
                        min_cost = cost
                        best_bs = bs
                        best_new_pos = new_pos
        elif METHOD == 'random':
            best_bs = random.choice(base_stations)
            if best_bs.is_in_coverage(curr_pos):
                best_new_pos = None
            else:
                best_new_pos = self.get_minimal_move_point(curr_pos, best_bs.position)
            min_cost = self.calculate_cost(best_bs, start_pos_idx, end_pos_idx, curr_pos, best_new_pos)
        elif METHOD == 'distance':
            # 选择距离最近的基站作为selected_bs
            min_dist = float('inf')
            for bs in base_stations:
                dist = calculate_3d_distance(self.position, bs.position)
                if dist < min_dist:
                    min_dist = dist
                    best_bs = bs
            if best_bs.is_in_coverage(curr_pos):
                best_new_pos = None
            else:
                best_new_pos = self.get_minimal_move_point(curr_pos, best_bs.position)
            min_cost = self.calculate_cost(best_bs, start_pos_idx, end_pos_idx, curr_pos, best_new_pos)
        elif METHOD == 'price':
            # 选择价格最低的基站作为selected_bs
            min_price = float('inf')
            for bs in base_stations:
                if bs.price_comp + bs.price_trans < min_price:
                    min_price = bs.price_comp + bs.price_trans
                    best_bs = bs
            if best_bs.is_in_coverage(curr_pos):
                best_new_pos = None
            else:
                best_new_pos = self.get_minimal_move_point(curr_pos, best_bs.position)
            min_cost = self.calculate_cost(best_bs, start_pos_idx, end_pos_idx, curr_pos, best_new_pos)
        else:
            raise ValueError('Invalid method')
        
        self.base_station = best_bs
        return best_bs, min_cost, best_new_pos

    def get_minimal_move_point(self, curr_pos, base_station_pos):
        """
        返回值：返回移动到刚好进入覆盖范围时的三维坐标，新位置的z保持不变。
        """
        # 将输入转换为numpy数组，确保数值计算有效
        drone_pos = np.array(curr_pos, dtype=float)
        base_station_pos = np.array(base_station_pos, dtype=float)
        
        # 计算垂直高度差
        vertical_diff = HEIGHT_DRONE - HEIGHT_BASE_STATION
        
        # 计算在固定高度下的水平距离临界值，使得3D距离正好为 coverage_range
        max_horizontal = np.sqrt(COVERAGE_RADIUS**2 - vertical_diff**2)
        
        # 提取二维平面坐标 (x, y)
        drone_horizontal = drone_pos[:2]
        base_horizontal = base_station_pos[:2]

        current_2d_distance = np.linalg.norm(drone_horizontal - base_horizontal)
        
        # 沿无人机与基站连线方向，将无人机移动到覆盖边界上的新水平坐标
        # 沿无人机与基站的方向，计算移动后的位置
        direction = (drone_horizontal - base_horizontal) / current_2d_distance
        new_horizontal = base_horizontal + direction * max_horizontal

        # 保持无人机原有的高度
        new_pos = [new_horizontal[0], new_horizontal[1], curr_pos[2]]
        return new_pos

    def reset_request_history(self):
        self.request_history = []
    
    def reset_battery(self):
        self.battery = BATTERY_CAPACITY
    
    def reset_trans_demand(self):
        self.trans_demand = 0