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

def calculate_distance(pos1, pos2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(pos1[:2], pos2[:2])))

def random_position(size):
    return [random.uniform(0, s) for s in size]

