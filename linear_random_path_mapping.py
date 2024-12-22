"""

"""
import numpy as np
import matplotlib.pyplot as plt
from multi_position_mapping import MultiPositionMapping
from eight_sonars_robot_single_mapping import RobotWithEightSonars
import os

def generate_measurement_positions(start=(2, 5), end=(28.5, 5), num_points=40, y_noise=0.4):
    """
    在直线路径上生成随机分布的测量位置，并添加y轴方向的随机误差
    
    参数:
    start (tuple): 起始位置 (x, y)
    end (tuple): 终止位置 (x, y)
    num_points (int): 需要生成的点数
    y_noise (float): y轴方向的最大随机误差范围（±y_noise）
    
    返回:
    list: 测量位置列表，按距离起点的距离排序
    """
    # 计算方向向量
    direction = np.array(end) - np.array(start)
    distance = np.linalg.norm(direction)
    unit_direction = direction / distance
    
    # 生成随机距离（均匀分布）
    random_distances = np.random.uniform(0, distance, num_points)
    random_distances.sort()  # 按距离排序，确保机器人从近到远移动
    
    # 生成y轴随机误差
    y_errors = np.random.uniform(-y_noise, y_noise, num_points)
    
    # 生成测量位置
    positions = []
    for t, y_err in zip(random_distances, y_errors):
        # 计算基础位置
        base_position = np.array(start) + t * unit_direction
        # 添加y轴误差
        position = base_position + np.array([0, y_err])
        positions.append(tuple(position))
    
    return positions

def load_and_process_map():
    """
    加载和处理地图
    """
    try:
        # 加载地图
        map_path = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\files\room_map.npy"
        map_data = np.load(map_path, allow_pickle=True).item()
        
        # 打印地图信息
        print("Map loaded successfully:")
        print(f"Resolution: {map_data['resolution']}m")
        print(f"Origin: {map_data['origin']}")
        print(f"Width: {map_data['width']}m")
        print(f"Length: {map_data['length']}m")
        print(f"Map shape: {map_data['map'].shape}")
        
        return map_data['map'], (map_data['length'], map_data['width']), map_data['resolution']
        
    except Exception as e:
        print(f"Error loading map: {str(e)}")
        print("Current working directory:", os.getcwd())
        raise

def calculate_orientation(previous_position, current_position):
    """
    计算机器人朝向
    
    参数:
    previous_position (tuple): 上一个测量点 (x, y)
    current_position (tuple): 当前测量点 (x, y)
    
    返回:
    float: 机器人朝向（弧度）
    """
    # 计算向量
    vector = np.array(current_position) - np.array(previous_position)
    # 计算朝向（使用arctan2以获得正确的象限）
    orientation = np.arctan2(vector[1], vector[0])
    return orientation

class RandomPathMapping(MultiPositionMapping):
    """
    继承MultiPositionMapping类，添加随机路径测量和可视化功能
    """
    def combine_measurements_progressively(self, robot_positions):
        """
        渐进式进行测量并保存每次测量的结果
        """
        previous_position = None
        for i, position in enumerate(robot_positions):
            # 计算机器人的朝向
            if previous_position is not None:
                orientation = self.calculate_orientation(previous_position, position)
            else:
                orientation = 0  # 第一个位置的初始朝向可以设为0
            
            # 创建机器人实例，设置初始位置和朝向
            robot = RobotWithEightSonars(position=position, orientation=orientation)
            
            # 使用真实地图进行测量模拟
            measurements = robot.simulate_measurements(self.real_map, self.grid_resolution)
            
            # 获取该位置的概率地图
            occupancy_map, empty_map, map_origin = robot.create_probability_maps(
                measurements,
                grid_resolution=self.grid_resolution
            )
            
            # 更新地图
            self.update_maps(occupancy_map, empty_map, position)
            
            # 保存当前状态
            self.save_current_state(robot_positions[:i+1], i+1, self.real_map)
            
            previous_position = position  # 更新上一个位置
    
    def update_maps(self, new_occupancy, new_empty, robot_position):
        """
        更新地图，将新的观测结果整合到现有地图中
        """
        # 计算新地图在全局坐标系中的范围
        map_origin = (robot_position[0] - 5, robot_position[1] - 5)  # 10m×10m的地图，机器人在中心
        
        # 计算需要更新的范围（裁剪到房间大小）
        x_min = max(0, map_origin[0])
        x_max = min(self.room_size[0], map_origin[0] + 10)
        y_min = max(0, map_origin[1])
        y_max = min(self.room_size[1], map_origin[1] + 10)
        
        # 计算局部地图中对应的索引
        ix_min = int((x_min - map_origin[0]) / self.grid_resolution)
        ix_max = int((x_max - map_origin[0]) / self.grid_resolution)
        iy_min = int((y_min - map_origin[1]) / self.grid_resolution)
        iy_max = int((y_max - map_origin[1]) / self.grid_resolution)
        
        # 计算全局地图中对应的索引
        gx_min = int(x_min / self.grid_resolution)
        gx_max = int(x_max / self.grid_resolution)
        gy_min = int(y_min / self.grid_resolution)
        gy_max = int(y_max / self.grid_resolution)
        
        # 确保索引有效
        ix_min = max(0, ix_min)
        ix_max = min(new_occupancy.shape[1], ix_max)
        iy_min = max(0, iy_min)
        iy_max = min(new_occupancy.shape[0], iy_max)
        
        gx_min = max(0, gx_min)
        gx_max = min(self.occupancy_map.shape[1], gx_max)
        gy_min = max(0, gy_min)
        gy_max = min(self.occupancy_map.shape[0], gy_max)
        
        # 确保提取的区域大小相同
        width = min(ix_max - ix_min, gx_max - gx_min)
        height = min(iy_max - iy_min, gy_max - gy_min)
        
        if width <= 0 or height <= 0:
            return  # 如果没有重叠区域，直接返回
        
        # 提取需要更新的区域
        local_occupancy = new_occupancy[iy_min:iy_min+height, ix_min:ix_min+width]
        local_empty = new_empty[iy_min:iy_min+height, ix_min:ix_min+width]
        
        # 更新占用地图（使用概率加法公式）
        self.occupancy_map[gy_min:gy_min+height, gx_min:gx_min+width] = np.maximum(
            self.occupancy_map[gy_min:gy_min+height, gx_min:gx_min+width],
            local_occupancy
        )
        
        # 更新空置地图（取最小值，因为是负值）
        self.empty_map[gy_min:gy_min+height, gx_min:gx_min+width] = np.minimum(
            self.empty_map[gy_min:gy_min+height, gx_min:gx_min+width],
            local_empty
        )
    
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
            # 绘制机器人路径
            path_x = [pos[0] for pos in positions_so_far]
            path_y = [pos[1] for pos in positions_so_far]
            ax.plot(path_x, path_y, 'g--', linewidth=1, label='Robot Path')
            # 绘制历史位置
            ax.plot(path_x[:-1], path_y[:-1], 'ko', markersize=5, label='Previous Positions')
            # 绘制当前位置
            current_pos = positions_so_far[-1]
            ax.plot(current_pos[0], current_pos[1], 'ro', markersize=8, label='Current Position')
            # 添加图例
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                     ncol=3, fancybox=True, shadow=True)
        
        plt.suptitle(f'Probability Maps - Step {step_number}', fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.2)
        
        # 保存概率地图
        prob_map_path = fr"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\random_orientation_probability_maps_step_{step_number:03d}.png"
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
                elif empty > 0.2:  # 低确定性空区域
                    ax4.plot(x, y, 'g+', markersize=4, markeredgewidth=1, label='Empty (Low Certainty)' if i==0 and j==0 else "")
                else:  # 未知区域
                    ax4.plot(x, y, 'k.', markersize=3, label='Unknown' if i==0 and j==0 else "")
        
        # 绘制机器人路径和位置
        path_x = [pos[0] for pos in positions_so_far]
        path_y = [pos[1] for pos in positions_so_far]
        ax4.plot(path_x, path_y, 'g--', linewidth=1, label='Robot Path')
        
        # 绘制机器人位置
        for pos in positions_so_far[:-1]:
            ax4.plot(pos[0], pos[1], 'ko', markersize=8, fillstyle='none', label='Previous Position' if pos==positions_so_far[0] else "")
        current_pos = positions_so_far[-1]
        ax4.plot(current_pos[0], current_pos[1], 'ro', markersize=8, fillstyle='none', label='Current Position')
        
        ax4.grid(True)
        
        # 将图例放在图片下方
        ax4.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                  ncol=3, fancybox=True, shadow=True)
        
        plt.title(f'Symbolic Sonar Map - Step {step_number}', fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.2)
        
        # 保存符号化地图
        symbolic_map_path = fr"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\linear_reconstruction\random_orentation_symbolic_map_step_{step_number:03d}.png"
        plt.savefig(symbolic_map_path, dpi=300, bbox_inches='tight')
        print(f"Step {step_number} saved to:\n{prob_map_path}\n{symbolic_map_path}")
        
        plt.close(fig2)

def main():
    # 设置随机种子以确保结果可重复
    np.random.seed(42)
    
    # 加载地图
    room_map, map_size, resolution = load_and_process_map()
    print(f"Loaded map with size: {map_size}, resolution: {resolution}m")
    
    # 创建渐进式建图实例，只传入必要的参数
    mapper = RandomPathMapping(grid_resolution=resolution, real_map=room_map)
    
    # 生成带有y轴随机误差的测量位置
    measurement_positions = generate_measurement_positions(start=(2, 5), end=(28, 5), y_noise=0.5)
    print(f"Generated {len(measurement_positions)} measurement positions")
    
    # 进行渐进式测量和保存
    mapper.combine_measurements_progressively(measurement_positions)
    
    print("Mapping complete!")

if __name__ == "__main__":
    main() 