import numpy as np
import matplotlib.pyplot as plt
from single_sonar_simulation import SingleUltrasonicSensor

class RobotWithEightSonars:
    def __init__(self, position=(2, 2), orientation=0):
        """
        初始化带有8个超声波传感器的机器人
        
        参数:
        position (tuple): 机器人位置 (x, y)
        orientation (float): 机器人朝向（弧度）
        """
        self.position = np.array(position)
        # 确保初始朝向在 -π 到 π 之间
        self.orientation = self.normalize_angle(orientation)
        
        # 定义8个传感器的相对角度（弧度）
        angles = np.array([0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 
                         -3*np.pi/4, -np.pi/2, -np.pi/4])
        self.sensors = [(i, angle) for i, angle in enumerate(angles)]

    @staticmethod
    def normalize_angle(angle):
        """
        将角度标准化到 -π 到 π 之间
        """
        # 使用取模运算将角度限制在 -2π 到 2π 之间
        angle = angle % (2 * np.pi)
        # 如果角度大于 π，减去 2π
        if angle > np.pi:
            angle -= 2 * np.pi
        # 如果角度小于 -π，加上 2π
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def simulate_measurements(self, real_map, resolution, noise_level=0.1):
        """
        模拟超声波传感器的测量，使用扇形检测范围
        """
        measurements = []
        beam_width = np.pi/6  # 30度的波束宽度
        num_beams = 5  # 在扇形区域内的射线数量
        
        for sensor_id, sensor_angle in self.sensors:
            # 计算传感器的全局角度（机器人朝向 + 传感器相对角度）
            global_angle = self.normalize_angle(self.orientation + sensor_angle)
            
            # 在扇形范围内发射多条射线
            min_distance = float('inf')
            start_angle = global_angle - beam_width/2
            angle_step = beam_width / (num_beams - 1)
            
            for i in range(num_beams):
                beam_angle = self.normalize_angle(start_angle + i * angle_step)
                direction = np.array([
                    np.cos(beam_angle),
                    np.sin(beam_angle)
                ])
                
                # 发射光线并计算交点
                intersection = self.trace_ray(self.position, direction, real_map, resolution)
                
                if intersection is not None:
                    # 计算距离
                    distance = np.linalg.norm(intersection - self.position)
                    min_distance = min(min_distance, distance)
            
            # 如果在扇形范围内检测到障碍物
            if min_distance < float('inf'):
                # 添加随机噪声
                noise = np.random.normal(0, noise_level)
                noisy_distance = max(0.0, min_distance + noise)
                measurements.append(noisy_distance)
            else:
                measurements.append(float('inf'))
        
        return measurements

    def set_orientation(self, new_orientation):
        """
        设置机器人的新朝向，并确保角度在正确范围内
        """
        self.orientation = self.normalize_angle(new_orientation)

    def measure_environment(self, room_size=(4, 4), wall_thickness=0.1):
        """
        模拟在给定环境中进行测量
        """
        measurements = []
        max_range = 5.0  # 设置最大测量范围
        
        # 定义墙壁位置
        walls = [
            # 上墙
            ([wall_thickness, room_size[1]-wall_thickness], 
             [room_size[0]-wall_thickness, room_size[1]-wall_thickness]),
            # 右墙
            ([room_size[0]-wall_thickness, wall_thickness], 
             [room_size[0]-wall_thickness, room_size[1]-wall_thickness]),
            # 下墙
            ([wall_thickness, wall_thickness], 
             [room_size[0]-wall_thickness, wall_thickness]),
            # 左墙
            ([wall_thickness, wall_thickness], 
             [wall_thickness, room_size[1]-wall_thickness])
        ]
        
        # 对每个传感器进行测量
        for sensor_id, sensor_angle in self.sensors:
            # 计算传感器的全局角度
            global_angle = self.normalize_angle(self.orientation + sensor_angle)
            
            # 在扇形范围内发射多条射线
            min_distance = float('inf')
            beam_width = np.pi/6  # 30度的波束宽度
            num_beams = 5  # 在扇形区域内的射线数量
            
            start_angle = global_angle - beam_width/2
            angle_step = beam_width / (num_beams - 1)
            
            for i in range(num_beams):
                beam_angle = self.normalize_angle(start_angle + i * angle_step)
                direction = np.array([
                    np.cos(beam_angle),
                    np.sin(beam_angle)
                ])
                
                # 检查与每面墙的交点
                for wall_idx, (wall_start, wall_end) in enumerate(walls):
                    wall_start = np.array(wall_start)
                    wall_end = np.array(wall_end)
                    
                    # 计算墙的向量和法向量
                    wall_vector = wall_end - wall_start
                    wall_normal = np.array([-wall_vector[1], wall_vector[0]])
                    wall_normal = wall_normal / np.linalg.norm(wall_normal)
                    
                    # 检查射线是否平行于墙
                    denominator = np.dot(direction, wall_normal)
                    if abs(denominator) > 1e-6:
                        # 计算射线参数 t
                        t = np.dot(wall_start - self.position, wall_normal) / denominator
                        
                        # 如果 t 为正，说明交点在射线前方
                        if t > 0:
                            # 计算交点
                            intersection = self.position + t * direction
                            
                            # 检查交点是否在墙段上
                            wall_min = np.minimum(wall_start, wall_end)
                            wall_max = np.maximum(wall_start, wall_end)
                            
                            if (intersection[0] >= wall_min[0] - 1e-6 and 
                                intersection[0] <= wall_max[0] + 1e-6 and
                                intersection[1] >= wall_min[1] - 1e-6 and 
                                intersection[1] <= wall_max[1] + 1e-6):
                                
                                distance = np.linalg.norm(intersection - self.position)
                                min_distance = min(min_distance, distance)
            
            # 添加随机误差
            if min_distance < float('inf'):
                noise = np.random.uniform(-0.1, 0.1)  # ±10cm的随机误差
                measured_distance = min_distance + noise
                measurements.append(measured_distance)
            else:
                measurements.append(float('inf'))
        
        return measurements

    def create_probability_maps(self, measurements, grid_resolution=0.1):
        """
        根据测量结果创建概率地图，使用与single_sonar_simulation相同的概率分布
        """
        # 创建10m×10m的地图，机器人在中心位置
        map_size = int(10 / grid_resolution)
        occupancy_map = np.zeros((map_size, map_size))
        empty_map = np.zeros((map_size, map_size))
        map_origin = (self.position[0] - 5, self.position[1] - 5)  # 地图原点坐标
        
        beam_width = np.pi/6  # 30度的波束宽度
        max_error = 0.1  # 最大测量误差
        R_min = 0.1  # 最小测量范围
        
        # 为每个传感器创建概率地图
        for (_, sensor_angle), R in zip(self.sensors, measurements):
            if R == float('inf'):
                continue
            
            # 计算传感器的全局角度
            global_angle = self.normalize_angle(self.orientation + sensor_angle)
            
            # 计算测量点的位置（用于可视化）
            end_point = np.array([
                self.position[0] + R * np.cos(global_angle),
                self.position[1] + R * np.sin(global_angle)
            ])
            
            # 更新概率地图
            for y in range(map_size):
                for x in range(map_size):
                    # 转换回世界坐标
                    world_x = x * grid_resolution + map_origin[0]
                    world_y = y * grid_resolution + map_origin[1]
                    point = np.array([world_x, world_y])
                    
                    # 计算点到机器人的向量
                    to_point = point - self.position
                    delta = np.linalg.norm(to_point)
                    if delta == 0:
                        continue
                    
                    # 计算角度差
                    point_angle = np.arctan2(to_point[1], to_point[0])
                    theta = abs(self.normalize_angle(point_angle - global_angle))
                    
                    # 如果角度太大，跳过
                    if theta > beam_width/2:
                        continue
                    
                    # 计算空置概率（距离在测量值之前）
                    if delta < R - max_error:
                        E_r = np.maximum(0, 1 - ((delta - R_min)/(R - max_error - R_min))**2)
                        E_a = np.maximum(0, 1 - (2*theta/beam_width)**2)
                        empty_prob = E_r * E_a
                        empty_map[y, x] = min(empty_map[y, x], -empty_prob)
                    
                    # 计算占用概率（距离在测量值附近）
                    elif R - max_error <= delta <= R + max_error:
                        O_r = np.maximum(0, 1 - ((delta - R)/max_error)**2)
                        O_a = np.maximum(0, 1 - (2*theta/beam_width)**2)
                        occ_prob = O_r * O_a
                        occupancy_map[y, x] = max(occupancy_map[y, x], occ_prob)
        
        return occupancy_map, empty_map, map_origin

    def visualize_maps(self, occupancy_map, empty_map, room_size=(4, 4), wall_thickness=0.1, save_path=None):
        """
        可视化占用地图和空置地图，只显示房间范围内的部分
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 获取地图的实际范围（10m×10m）
        map_origin = (self.position[0] - 5, self.position[1] - 5)
        
        # 计算需要显示的范围（裁剪到房间大小）
        display_x_min = max(0, map_origin[0])
        display_x_max = min(room_size[0], map_origin[0] + 10)
        display_y_min = max(0, map_origin[1])
        display_y_max = min(room_size[1], map_origin[1] + 10)
        
        # 计算对应的数��索引
        grid_resolution = 0.1
        ix_min = max(0, int((display_x_min - map_origin[0]) / grid_resolution))
        ix_max = min(occupancy_map.shape[1], int((display_x_max - map_origin[0]) / grid_resolution))
        iy_min = max(0, int((display_y_min - map_origin[1]) / grid_resolution))
        iy_max = min(occupancy_map.shape[0], int((display_y_max - map_origin[1]) / grid_resolution))
        
        # 裁剪地图
        occupancy_display = occupancy_map[iy_min:iy_max, ix_min:ix_max]
        empty_display = empty_map[iy_min:iy_max, ix_min:ix_max]
        
        # 显示范围
        display_extent = [display_x_min, display_x_max, display_y_min, display_y_max]
        
        # 绘制占用地图
        im1 = ax1.imshow(occupancy_display, origin='lower', 
                         extent=display_extent, cmap='Reds')
        ax1.set_title('Occupancy Map')
        plt.colorbar(im1, ax=ax1)
        
        # 绘制空置地图
        im2 = ax2.imshow(empty_display, origin='lower', 
                         extent=display_extent, cmap='Blues')
        ax2.set_title('Empty Map')
        plt.colorbar(im2, ax=ax2)
        
        # 在两个图上都标注机器人位置
        ax1.plot(self.position[0], self.position[1], 'ko', markersize=10, label='Robot')
        ax2.plot(self.position[0], self.position[1], 'ko', markersize=10, label='Robot')
        
        # 绘制墙壁
        walls = [
            # 上墙
            ([wall_thickness, room_size[1]-wall_thickness], 
             [room_size[0]-wall_thickness, room_size[1]-wall_thickness]),
            # 右墙
            ([room_size[0]-wall_thickness, wall_thickness], 
             [room_size[0]-wall_thickness, room_size[1]-wall_thickness]),
            # 下墙
            ([wall_thickness, wall_thickness], 
             [room_size[0]-wall_thickness, wall_thickness]),
            # 左墙
            ([wall_thickness, wall_thickness], 
             [wall_thickness, room_size[1]-wall_thickness])
        ]
        
        for wall_start, wall_end in walls:
            ax1.plot([wall_start[0], wall_end[0]], 
                    [wall_start[1], wall_end[1]], 
                    'k-', linewidth=2)
            ax2.plot([wall_start[0], wall_end[0]], 
                    [wall_start[1], wall_end[1]], 
                    'k-', linewidth=2)
        
        # 设置坐标轴范围为房间大小
        for ax in [ax1, ax2]:
            ax.set_xlim(0, room_size[0])
            ax.set_ylim(0, room_size[1])
            ax.grid(True)
            ax.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Figure saved to: {save_path}")
        
        plt.show()

    def simulate_measurements(self, real_map, resolution, noise_level=0.1):
        """
        模拟超声波传感器的测量，并在测量到的位置添加随机噪声
        
        参数:
        real_map: 真实环境地图
        resolution: 网格分辨率
        noise_level (float): 随机噪声的标准差（米）
        
        返回:
        measurements: 超声波测量结果（距离列表）
        """
        measurements = []
        for sensor, angle in self.sensors:
            # 计算传感器的发射方向
            direction = np.array([
                np.cos(self.orientation + angle),  # 考虑机器人的朝向和传感器的角度
                np.sin(self.orientation + angle)
            ])
            
            # 发射光线并计算交点
            intersection = self.trace_ray(self.position, direction, real_map, resolution)
            
            # 记录测量结果
            if intersection is not None:
                # 计算基础距离
                distance = np.linalg.norm(intersection - self.position)
                
                # 添加随机噪声到距离
                noise = np.random.normal(0, noise_level)
                noisy_distance = distance + noise
                
                # 确保距离不为负
                noisy_distance = max(0.0, noisy_distance)
                
                measurements.append(noisy_distance)
            else:
                measurements.append(float('inf'))  # 如果没有交点，记录为无穷大
        
        return measurements

    def trace_ray(self, origin, direction, real_map, resolution):
        """
        追踪光线与地图的交点
        
        参数:
        origin: 光线起始点
        direction: 光线方向
        real_map: 真实环境地图
        resolution: 网格分辨率
        
        返回:
        intersection: 光线与地图的交点
        """
        # 确保 current_position 是浮点数类型的 numpy 数组
        current_position = np.array(origin, dtype=np.float64)
        step_size = 0.1  # 可以根据需要调整步长
        
        while True:
            # 将当前坐标转换为网格坐标
            grid_x = int(current_position[0] / resolution)
            grid_y = int(current_position[1] / resolution)
            
            # 检查是否超出地图边界
            if grid_x < 0 or grid_x >= real_map.shape[1] or grid_y < 0 or grid_y >= real_map.shape[0]:
                return None  # 超出边界，返回无交点
            
            # 检查当前网格是否为障碍物
            if real_map[grid_y, grid_x] == 1:  # 假设1表示障碍物
                return current_position  # 返回交点
            
            # 更新当前坐标（保持浮点数类型）
            current_position = current_position + direction * step_size
            
        return None  # 如果没有交点，返回无交点

def main():
    # 创建机器人实例（在房间中心）
    robot = RobotWithEightSonars()
    
    # 进行一次测量
    measurements = robot.measure_environment()
    
    # 创建概率地图
    occupancy_map, empty_map, map_origin = robot.create_probability_maps(measurements)
    
    # 可视化结果并保存
    save_path = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\sonar_mapping_result3.png"
    robot.visualize_maps(occupancy_map, empty_map, save_path=save_path)

if __name__ == "__main__":
    main() 