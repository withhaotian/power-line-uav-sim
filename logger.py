"""
9. Logger.py - 数据记录
功能
记录仿真数据（如无人机位置、任务状态）。
"""

class Logger:
    def __init__(self):
        self.logs = []

    def log(self, step, drones, power_stations, base_stations):
        log_entry = f"Step {step}:\n"
        for drone in drones:
            log_entry += f"Drone {drone.drone_id}: Pos={drone.position}, Battery={drone.battery}, Task={drone.task.task_id if drone.task else None}\n"
        self.logs.append(log_entry)

    def save(self, filename):
        with open(filename, "w") as f:
            f.write("\n".join(self.logs))