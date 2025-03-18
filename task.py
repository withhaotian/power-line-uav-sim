"""
2. Task.py - 任务类
功能
定义巡检任务的属性，管理任务状态。

内容
属性：
    task_id：任务ID
    target：巡检目标（PowerStation 对象）
    status：任务状态（"pending"、"assigned"、"completed"）
方法：
    update_status(status)：更新任务状态
"""

class Task:
    def __init__(self, task_id, target):
        self.task_id = task_id
        self.target = target  # PowerStation对象
        self.status = "pending"

    def update_status(self, status):
        self.status = status