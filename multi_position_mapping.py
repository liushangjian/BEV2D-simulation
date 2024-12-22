"""
多位置测量和融合
这个代码是自定义一个写死的地图作为测试的
"""
import numpy as np
import matplotlib.pyplot as plt
from eight_sonars_robot_single_mapping import RobotWithEightSonars
import os

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

class MultiPositionMapping:
    def __init__(self, grid_resolution=0.1, real_map=None):
        """
        初始化多位置建图系统
        
        参数:
        grid_resolution (float): 网格分辨率
        real_map: 真实环境地图
        """
        self.grid_resolution = grid_resolution
        self.real_map = real_map
        
        # 根据真实地图确定房间尺寸
        if real_map is not None:
            height, width = real_map.shape
            self.room_size = (width * grid_resolution, height * grid_resolution)
        else:
            raise ValueError("real_map cannot be None")
        
        # 创建网格
        self.x = np.arange(0, self.room_size[0] + grid_resolution, grid_resolution)
        self.y = np.arange(0, self.room_size[1] + grid_resolution, grid_resolution)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        
        # 初始化概率地图
        self.occupancy_map = np.zeros_like(self.X)  # 占用概率地图
        self.empty_map = np.zeros_like(self.X)      # 空置概率地图
        
    def update_maps(self, occupancy_map, empty_map, robot_pos):
        """
        更新概率地图，使用概率叠加公式
        
        参数:
        occupancy_map: 新的占用概率地图
        empty_map: 新的空置概率地图
        robot_pos: 机器人位置
        """
        # 计算局部地图在全局地图中的位置
        gx_min = int((robot_pos[0] - 5) / self.grid_resolution)
        gy_min = int((robot_pos[1] - 5) / self.grid_resolution)
        gx_max = gx_min + occupancy_map.shape[1]
        gy_max = gy_min + occupancy_map.shape[0]
        
        print(f"\nDebug update_maps:")
        print(f"Initial robot_pos: {robot_pos}")
        print(f"Initial bounds: x[{gx_min}:{gx_max}], y[{gy_min}:{gy_max}]")
        print(f"Initial occupancy_map shape: {occupancy_map.shape}")
        print(f"Global map shape: {self.occupancy_map.shape}")
        
        # 确保边界不超出全局地图范围
        if gx_min < 0:
            print(f"Adjusting gx_min from {gx_min} to 0")
            occupancy_map = occupancy_map[:, -gx_min:]
            empty_map = empty_map[:, -gx_min:]
            gx_min = 0
        if gy_min < 0:
            print(f"Adjusting gy_min from {gy_min} to 0")
            occupancy_map = occupancy_map[-gy_min:, :]
            empty_map = empty_map[-gy_min:, :]
            gy_min = 0
        if gx_max > self.occupancy_map.shape[1]:
            diff = gx_max - self.occupancy_map.shape[1]
            print(f"Adjusting gx_max from {gx_max} to {self.occupancy_map.shape[1]}")
            occupancy_map = occupancy_map[:, :-diff]
            empty_map = empty_map[:, :-diff]
            gx_max = self.occupancy_map.shape[1]
        if gy_max > self.occupancy_map.shape[0]:
            diff = gy_max - self.occupancy_map.shape[0]
            print(f"Adjusting gy_max from {gy_max} to {self.occupancy_map.shape[0]}")
            occupancy_map = occupancy_map[:-diff, :]
            empty_map = empty_map[:-diff, :]
            gy_max = self.occupancy_map.shape[0]
        
        print(f"Final bounds: x[{gx_min}:{gx_max}], y[{gy_min}:{gy_max}]")
        print(f"Final occupancy_map shape: {occupancy_map.shape}")
        print(f"Update region shape: ({gy_max-gy_min}, {gx_max-gx_min})")
        
        # 获取需要更新的区域
        region = slice(gy_min, gy_max), slice(gx_min, gx_max)
        
        # 1. 更新空置区域（使用概率加法公式）
        # 确保空置值在 (-1, 0) 范围内
        empty_update = (
            self.empty_map[region] + 
            empty_map - 
            self.empty_map[region] * empty_map
        )
        self.empty_map[region] = np.clip(empty_update, -1, 0)
        
        # 2. 根据空置确定性因子削弱占用概率
        # 确保占用值在 (0, 1] 范围内
        occupancy_map = np.clip(
            occupancy_map * (1 - self.empty_map[region]),
            0,
            1
        )
        
        # 3. 更��占用区域（使用概率加法公式）
        # 确保占用值在 (0, 1] 范围内
        occupancy_update = (
            self.occupancy_map[region] + 
            occupancy_map - 
            self.occupancy_map[region] * occupancy_map
        )
        self.occupancy_map[region] = np.clip(occupancy_update, 0, 1)
        
        # 4. 处理未知区域（确保当两个值都为0时保持未知状态）
        unknown_mask = (self.empty_map[region] == 0) & (self.occupancy_map[region] == 0)
        self.empty_map[region][unknown_mask] = 0
        self.occupancy_map[region][unknown_mask] = 0
        
    def combine_measurements(self, robot_positions):
        """
        进行多位置测量和融合
        
        参数:
        robot_positions (list): 机器人位置列表 [(x1,y1), (x2,y2), ...]
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
            
            # 使用真实地图进行测量模���
            measurements = robot.simulate_measurements(self.real_map, self.grid_resolution)
            
            # 获取该位置的概率地图
            occupancy_map, empty_map, map_origin = robot.create_probability_maps(
                measurements,
                grid_resolution=self.grid_resolution
            )
            
            # 更新地图
            self.update_maps(occupancy_map, empty_map, position)
            
            previous_position = position  # 更新上一个位置
        
    def visualize_results(self, robot_positions, save_path=None):
        """
        可视化融合后的结果
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 绘制占用地图
        im1 = ax1.imshow(self.occupancy_map, origin='lower', 
                         extent=[0, self.room_size[0], 0, self.room_size[1]], 
                         cmap='Reds')
        ax1.set_title('Combined Occupancy Map')
        plt.colorbar(im1, ax=ax1)
        
        # 绘制空置地图
        im2 = ax2.imshow(self.empty_map, origin='lower', 
                         extent=[0, self.room_size[0], 0, self.room_size[1]], 
                         cmap='Blues')
        ax2.set_title('Combined Empty Map')
        plt.colorbar(im2, ax=ax2)
        
        # 在两个图上都标注机器人位置
        for pos in robot_positions:
            ax1.plot(pos[0], pos[1], 'ko', markersize=10, label=f'Robot at {pos}')
            ax2.plot(pos[0], pos[1], 'ko', markersize=10, label=f'Robot at {pos}')
        
        ax1.grid(True)
        ax2.grid(True)
        
        # 将图例放在右上角
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Figure saved to: {save_path}")
            
        plt.show()

    def calculate_orientation(self, previous_position, current_position):
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

def main():
    # 加载地图
    room_map, map_size, resolution = load_and_process_map()
    print(f"Loaded map with size: {map_size}, resolution: {resolution}m")
    
    # 创建多位置建图实例
    mapper = MultiPositionMapping(grid_resolution=resolution, real_map=room_map)
    
    # 定义机器人测量位置
    robot_positions = [(3, 5), (5, 5), (6, 5.5)]
    
    # 进行多位置测量和融合
    mapper.combine_measurements(robot_positions)
    
    # 可视化并保存结果
    save_path = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\multi_position_mapping_result3.png"
    mapper.visualize_results(robot_positions, save_path)

if __name__ == "__main__":
    main() 
    