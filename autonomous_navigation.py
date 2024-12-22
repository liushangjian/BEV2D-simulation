import numpy as np
from linear_path_mapping import MultiPositionMapping
from eight_sonars_robot_single_mapping import RobotWithEightSonars
from linear_random_path_mapping import load_and_process_map
import heapq
import matplotlib.pyplot as plt

class NavigationRobot(MultiPositionMapping):
    def __init__(self, grid_resolution=0.1, real_map=None):
        super().__init__(grid_resolution=grid_resolution, real_map=real_map)
        self.robot_pos = np.array([2.5, 7.0])  # 初始位置
        self.target_pos = np.array([29.0, 1.0])  # 目标位置
        self.robot_orientation = 0  # 初始朝向
        self.path = []  # 存储路径
        self.visited_positions = []  # 存储已访问的位置
    
    def distance_to_target(self):
        """计算到目标的距离"""
        return np.linalg.norm(self.robot_pos - self.target_pos)
    
    def get_valid_neighbors(self, current_pos):
        """获取当前位置的二级相邻格子，步长固定为0.4m"""
        neighbors = []
        step_size = 0.4  # 固定步长为0.4m
        
        print(f"\nDebug get_valid_neighbors:")
        print(f"Current position: {current_pos}")
        print(f"Map shapes - Occupancy: {self.occupancy_map.shape}, Empty: {self.empty_map.shape}")
        
        # 检查当前位置的地图状态（使用与update_maps相同的坐标转换）
        current_x = int(current_pos[0] / self.grid_resolution)
        current_y = int(current_pos[1] / self.grid_resolution)
        print(f"Current position map status:")
        print(f"Map coordinates: [{current_x}, {current_y}]")
        print(f"Occupancy: {self.occupancy_map[current_y, current_x]:.3f}")
        print(f"Empty: {self.empty_map[current_y, current_x]:.3f}")
        
        # 二级相邻方向
        directions = [
            (-2, -2), (-2, -1), (-2, 0), (-2, 1), (-2, 2),
            (-1, -2),                             (-1, 2),
            (0, -2),                              (0, 2),
            (1, -2),                              (1, 2),
            (2, -2),  (2, -1),  (2, 0),  (2, 1),  (2, 2)
        ]
        
        for dx, dy in directions:
            # 计算新位置，对斜向移动进行距离调整
            if dx != 0 and dy != 0:  # 斜向移动
                new_x = current_pos[0] + dx * step_size / 3
                new_y = current_pos[1] + dy * step_size / 3
            else:  # 直线移动
                new_x = current_pos[0] + dx * step_size
                new_y = current_pos[1] + dy * step_size
            
            # 检查是否在地图范围内
            if 0 <= new_x <= 30 and 0 <= new_y <= 10:
                is_path_clear = True
                
                # 只检查中点和终点
                check_points = [
                    # 中点
                    ((current_pos[0] + new_x) / 2, (current_pos[1] + new_y) / 2),
                    # 终点
                    (new_x, new_y)
                ]
                
                # 检查这两个关键点
                for i, (px, py) in enumerate(check_points):
                    point_type = "Midpoint" if i == 0 else "Endpoint"
                    
                    # 使用与update_maps相同的坐标转换
                    map_x = int(px / self.grid_resolution)
                    map_y = int(py / self.grid_resolution)
                    
                    
                    # 检查点是否在地图范围内
                    if (map_x < 0 or map_x >= self.occupancy_map.shape[1] or
                        map_y < 0 or map_y >= self.occupancy_map.shape[0]):
                        is_path_clear = False
                        break
                    
                    # 检查占用状态
                    occupancy = self.occupancy_map[map_y, map_x]
                    empty = self.empty_map[map_y, map_x]
                    
                    # 如果这个点还没有被探索过（都是0），我们应该允许通过
                    #if occupancy == 0 and empty == 0:
                    #    print("Unexplored area, allowing passage")
                    #    continue
                    
                    if occupancy >= 0.5:
                        is_path_clear = False
                        break
                    
                    if empty > -0.3:
                        is_path_clear = False
                        break
                
                if is_path_clear:
                    print(f"Path clear! Adding ({new_x:.3f}, {new_y:.3f}) to neighbors")
                    neighbors.append((new_x, new_y))
                else:
                    print("Path blocked")
        
        print(f"\nFound {len(neighbors)} valid neighbors")
        return neighbors
    
    def find_next_position(self):
        """使用最速接近法找到下一个最佳位置"""
        # 获取所有可能的相邻位置
        neighbors = self.get_valid_neighbors(self.robot_pos)
        if not neighbors:
            return None
        
        # 计算每个邻居到目标的距离
        distances = []
        for neighbor in neighbors:
            # 计算到目标的直线距离
            dist = np.linalg.norm(np.array(neighbor) - self.target_pos)
            distances.append((dist, neighbor))
        
        # 按距离排序
        distances.sort()  # 按距离升序排序
        
        # 返回距离目标最近的有效位置
        return np.array(distances[0][1])
    
    def navigate(self):
        """执行导航过程"""
        step = 0
        while self.distance_to_target() > 0.5:
            print(f"\n=== Step {step} ===")
            print(f"Current position: ({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f})")
            print(f"Distance to target: {self.distance_to_target():.2f}m")
            print(f"Target position: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f})")
            
            print("1. Measuring environment...")
            # 创建机器人实例并进行测量
            robot = RobotWithEightSonars(position=self.robot_pos, orientation=self.robot_orientation)
            measurements = robot.simulate_measurements(self.real_map, self.grid_resolution)
            
            print("2. Updating probability maps...")
            # 更新地图
            occupancy_map, empty_map, _ = robot.create_probability_maps(
                measurements,
                grid_resolution=self.grid_resolution
            )
            self.update_maps(occupancy_map, empty_map, self.robot_pos)

            print("3. Saving current state...")
            # 保存当前状态
            self.save_current_state(self.path, step + 1, self.real_map)
            
            step += 1
            print(f"Step {step} complete!")
            print(f"Current path: {[(round(pos[0], 1), round(pos[1], 1)) for pos in self.path]}")
            print("-" * 40)
                        
            # 记录路径
            self.path.append(self.robot_pos.copy())
            
            print("4. Planning next move...")
            # 找到下一个位置
            next_pos = self.find_next_position()
            if next_pos is None:
                print("ERROR: No valid path found!")
                print("Available neighbors:", self.get_valid_neighbors(self.robot_pos))
                break
            
            print(f"4. Moving to: ({next_pos[0]:.1f}, {next_pos[1]:.1f})")
            print(f"   Movement vector: ({(next_pos[0]-self.robot_pos[0]):.1f}, {(next_pos[1]-self.robot_pos[1]):.1f})")
            
            # 更新机器人位置和朝向
            direction = next_pos - self.robot_pos
            self.robot_orientation = np.arctan2(direction[1], direction[0])
            self.robot_pos = next_pos
            
            # 将位置四舍五入到最近的格点（精度0.1m）
       #     old_pos = self.robot_pos.copy()
       #     self.robot_pos = np.round(self.robot_pos / 0.1) * 0.1
       #     print(f"   Position before rounding: ({old_pos[0]:.1f}, {old_pos[1]:.1f})")
      #      print(f"   Position after rounding: ({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f})")           
            if step > 200:  # 添加大步数限制
                print("Maximum steps reached!")
                break
        
        if self.distance_to_target() <= 0.5:
            print("\n=== Navigation Complete! ===")
            print(f"Target reached in {step} steps!")
            print(f"Final position: ({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f})")
            print(f"Distance to target: {self.distance_to_target():.2f}m")
        return self.path
    
    def save_current_state(self, positions_so_far, step_number, real_map):
        """
        保存当前测量状态，分别保存概率地图和符号化地图
        """
        # 保存前三张子图
        fig1, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6))
        
        # 绘制真实地图
        im0 = ax1.imshow(real_map, origin='lower', cmap='gray_r',
                        extent=[0, self.room_size[0], 0, self.room_size[1]])
        ax1.set_title('Real Environment Map')
        plt.colorbar(im0, ax=ax1)
        
        # 绘制占用地图
        im1 = ax2.imshow(self.occupancy_map, origin='lower', 
                        extent=[0, self.room_size[0], 0, self.room_size[1]], 
                        cmap='Reds')
        ax2.set_title('Reconstructed Occupancy Map')
        plt.colorbar(im1, ax=ax2)
        
        # 绘制空置地图
        im2 = ax3.imshow(self.empty_map, origin='lower', 
                        extent=[0, self.room_size[0], 0, self.room_size[1]], 
                        cmap='Blues')
        ax3.set_title('Reconstructed Empty Map')
        plt.colorbar(im2, ax=ax3)
        
        # 为每个子图添加网格和路径标记
        for ax in [ax1, ax2, ax3]:
            ax.grid(True)
            # 绘制当前位置（即使没有路径历史）
            ax.plot(self.robot_pos[0], self.robot_pos[1], 'ro', markersize=8, 
                   fillstyle='none', label='Current Position')
            # 绘制目标位置
            ax.plot(self.target_pos[0], self.target_pos[1], 'g*', markersize=12, 
                   label='Target')
            
            # 如果有历史路径，则绘制
            if positions_so_far:
                # 绘制机器人路径
                path_x = [pos[0] for pos in positions_so_far]
                path_y = [pos[1] for pos in positions_so_far]
                ax.plot(path_x, path_y, 'g--', linewidth=1, label='Robot Path')
                # 绘制历史位置
                ax.plot(path_x, path_y, 'ko', markersize=5, label='Previous Positions')
            
            # 添加图例
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                     ncol=3, fancybox=True, shadow=True)
        
        plt.suptitle(f'Navigation Progress - Step {step_number}', fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.2)
        
        # 设置保存路径
        output_dir = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\auto_drive\basic"
        prob_map_path = fr"{output_dir}\autonomous_navigation_maps_step_{step_number:03d}.png"
        plt.savefig(prob_map_path, dpi=300, bbox_inches='tight')
        plt.close(fig1)
        
        # 创建并保存符号化地图
        fig2, ax4 = plt.subplots(figsize=(15, 5))
        ax4.set_title('Symbolic Sonar Map')
        
        # 设置坐标范围
        ax4.set_xlim(0, self.room_size[0])
        ax4.set_ylim(0, self.room_size[1])
        
        # 设置白色背景
        ax4.set_facecolor('white')
        fig2.patch.set_facecolor('white')
        
        # 首先绘制真实地图（使用反色显示）
        ax4.imshow(real_map, origin='lower', 
                   extent=[0, self.room_size[0], 0, self.room_size[1]],
                   cmap='gray_r', alpha=0.3)  # 使用gray_r进行反色显示
        
        # 使用0.5m的网格重新采样
        display_resolution = 0.2
        x_points = np.arange(0, self.room_size[0], display_resolution)
        y_points = np.arange(0, self.room_size[1], display_resolution)
        X, Y = np.meshgrid(x_points, y_points)
        
        # 计算每个显示网格包含的原始网格数量
        grid_ratio = int(display_resolution / self.grid_resolution)
        
        # 绘制不同类型的区域
        for i in range(len(x_points)):
            for j in range(len(y_points)):
                x_start = i * grid_ratio
                x_end = min((i + 1) * grid_ratio, self.occupancy_map.shape[1])
                y_start = j * grid_ratio
                y_end = min((j + 1) * grid_ratio, self.occupancy_map.shape[0])
                
                occupancy = np.mean(self.occupancy_map[y_start:y_end, x_start:x_end])
                empty = np.mean(abs(self.empty_map[y_start:y_end, x_start:x_end]))
                
                x, y = X[j,i], Y[j,i]
                
                if occupancy > 0.7:  # 被占用区域
                    ax4.plot(x, y, 'rx', markersize=8, label='Occupied' if i==0 and j==0 else "")
                elif empty > 0.7:  # 高确定性空区域
                    ax4.plot(x, y, 'b.', markersize=16, label='Empty (High Certainty)' if i==0 and j==0 else "")
                elif empty > 0.4:  # 中等确定性空区域
                    ax4.plot(x, y, 'c+', markersize=4, markeredgewidth=2, label='Empty (Medium Certainty)' if i==0 and j==0 else "")
                elif empty > 0.2:  # 低确定性空区��
                    ax4.plot(x, y, 'g+', markersize=4, markeredgewidth=1, label='Empty (Low Certainty)' if i==0 and j==0 else "")
                else:  # 未知区域
                    ax4.plot(x, y, 'k.', markersize=3, label='Unknown' if i==0 and j==0 else "")
        
        # 绘制机器人路径和位置
        if positions_so_far:
            path_x = [pos[0] for pos in positions_so_far]
            path_y = [pos[1] for pos in positions_so_far]
            ax4.plot(path_x, path_y, 'g--', linewidth=1, label='Robot Path')
            
            # 绘制历史位置
            for i, pos in enumerate(positions_so_far[:-1]):
                ax4.plot(pos[0], pos[1], 'ko', markersize=8, fillstyle='none', 
                        label='Previous Position' if i==0 else "")
        
        # 绘制当前位置
        ax4.plot(self.robot_pos[0], self.robot_pos[1], 'ro', markersize=8, fillstyle='none', 
                label='Current Position')
        
        # 绘制目标位置
        ax4.plot(self.target_pos[0], self.target_pos[1], 'g*', markersize=12, 
                label='Target')
        
        ax4.grid(True)
        
        # 将图例放在图片下方
        ax4.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                  ncol=3, fancybox=True, shadow=True)
        
        plt.title(f'Symbolic Sonar Map - Step {step_number}', fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.2)
        
        # 保存符号化地图
        symbolic_map_path = fr"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\auto_drive\reconstruction\autonomous_navigation_symbolic_map_step_{step_number:03d}.png"
        plt.savefig(symbolic_map_path, dpi=300, bbox_inches='tight')
        print(f"Step {step_number} saved to:\n{prob_map_path}\n{symbolic_map_path}")
        
        plt.close(fig2)
        
        
    def __del__(self):
       """析构函数，确保线程池被正确关闭"""
       if hasattr(self, 'executor'):
           self.executor.shutdown()

    def parallel_map_update(self):
        with Pool(processes=12) as pool:
            results = pool.map(self.process_measurement, measurements)

    def create_probability_maps(self, measurements, grid_resolution=0.1):
        # 使用numpy的广播和向量化操作替代显式循环
        X, Y = np.meshgrid(np.arange(map_size), np.arange(map_size))
        world_coords = np.stack([X * grid_resolution + map_origin[0], 
                               Y * grid_resolution + map_origin[1]], axis=-1)

def main():
    # 直接调用函数，而不是通过类调用
    room_map, map_size, resolution = load_and_process_map()
    print(f"Loaded map with size: {map_size}, resolution: {resolution}m")
    
    # 创建导航机器人实例
    navigator = NavigationRobot(grid_resolution=resolution, real_map=room_map)
    
    # 执行导航
    path = navigator.navigate()
    
    print("Navigation complete!")
    print(f"Final path length: {len(path)} steps")

if __name__ == "__main__":
    main()