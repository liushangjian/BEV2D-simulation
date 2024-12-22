"""
地图生成器，用于生成一个简单的房间地图，包括墙壁、墙边物体和中心物体。

增加了默认的物体数量从8个到16个
为每个墙边物体添加了随机的突出部分
物体的基础深度现在是随机的（2-4格）
突出部分有50%的概率出现，深度和宽度/高度都是随机的
这些修改会使地图更加复杂和真实，沿墙的物体会有更多的变化和细节。你可以通过调整以下参数来进一步定制地图：
num_objects: 控制物体的总数
min_size和max_size: 控制物体的基础大小范围

MIT License Copyright (c) 2024 刘商鉴
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom

class MapGenerator:
    def __init__(self, length=30, width=10, resolution=1.0):
        self.length = length  # 房间长度（米）
        self.width = width    # 房间宽度（米）
        self.resolution = resolution  # 地图分辨率（米/格）
        self.final_resolution = 0.1   # 最终保存时的分辨率（米/格）
        
        # 计算栅格数量（使用1m分辨率）
        self.grid_length = int(length / resolution)
        self.grid_width = int(width / resolution)
        
        # 初始化地图（0表示空闲，1表示占据）
        self.map = np.zeros((self.grid_width, self.grid_length), dtype=np.int8)
        
    def add_walls(self):
        """添加房间的墙壁"""
        # 上下墙
        self.map[0, :] = 1
        self.map[-1, :] = 1
        # 左右墙
        self.map[:, 0] = 1
        self.map[:, -1] = 1
        
    def add_wall_objects(self, min_size=2, max_size=4, num_objects=32):
        """在墙边添加连续的物体，包括一些突出部分。允许重叠并合并。"""
        for wall in ['top', 'bottom', 'left', 'right']:
            if wall in ['top', 'bottom']:
                positions = np.sort(np.random.choice(
                    range(1, self.grid_length-max_size-1), 
                    num_objects//4, 
                    replace=True
                ))
                for pos in positions:
                    size = np.random.randint(min_size, max_size)
                    depth = np.random.randint(2, 3)
                    
                    if wall == 'top':
                        temp_map = np.zeros_like(self.map)
                        temp_map[1:1+depth, pos:pos+size] = 1
                        self.map |= temp_map
                        
                        num_protrusions = np.random.randint(1, 4)
                        for _ in range(num_protrusions):
                            if np.random.random() > 0.2:
                                extra_depth = np.random.randint(1, 2)
                                extra_width = np.random.randint(1, max(2, min(4, size))) if size > 1 else 1
                                if extra_width < size:
                                    start_x = pos + np.random.randint(0, size-extra_width)
                                    temp_map = np.zeros_like(self.map)
                                    temp_map[1:1+depth+extra_depth, start_x:start_x+extra_width] = 1
                                    self.map |= temp_map
                    
                    else:  # bottom wall
                        temp_map = np.zeros_like(self.map)
                        temp_map[-1-depth:-1, pos:pos+size] = 1
                        self.map |= temp_map
                        
                        num_protrusions = np.random.randint(1, 4)
                        for _ in range(num_protrusions):
                            if np.random.random() > 0.2:
                                extra_depth = np.random.randint(1, 2)
                                extra_width = np.random.randint(1, max(2, min(4, size))) if size > 1 else 1
                                if extra_width < size:
                                    start_x = pos + np.random.randint(0, size-extra_width)
                                    temp_map = np.zeros_like(self.map)
                                    temp_map[-1-depth-extra_depth:-1, start_x:start_x+extra_width] = 1
                                    self.map |= temp_map
            
            else:  # left or right walls
                positions = np.sort(np.random.choice(
                    range(2, self.grid_width-max_size-2), 
                    num_objects//4, 
                    replace=True
                ))
                for pos in positions:
                    size = np.random.randint(min_size//2, max_size//2)
                    depth = np.random.randint(2, 3)
                    
                    if wall == 'left':
                        temp_map = np.zeros_like(self.map)
                        temp_map[pos:pos+size, 1:1+depth] = 1
                        self.map |= temp_map
                        
                        if np.random.random() > 0.2 and size > 2:
                            extra_depth = np.random.randint(1, 2)
                            extra_height = np.random.randint(1, max(2, min(4, size))) if size > 1 else 1
                            if extra_height < size:
                                start_y = pos + np.random.randint(0, size-extra_height)
                                temp_map = np.zeros_like(self.map)
                                temp_map[start_y:start_y+extra_height, 1:1+depth+extra_depth] = 1
                                self.map |= temp_map
                    
                    else:  # right wall
                        temp_map = np.zeros_like(self.map)
                        temp_map[pos:pos+size, -1-depth:-1] = 1
                        self.map |= temp_map
                        
                        if np.random.random() > 0.2:
                            extra_depth = np.random.randint(1, 2)
                            extra_height = np.random.randint(1, max(2, min(4, size))) if size > 1 else 1
                            if extra_height < size:
                                start_y = pos + np.random.randint(0, size-extra_height)
                                temp_map = np.zeros_like(self.map)
                                temp_map[start_y:start_y+extra_height, -1-depth-extra_depth:-1] = 1
                                self.map |= temp_map
        
    def add_center_objects(self, num_objects=3):
        """在房间中间添加物体"""
        for _ in range(1):
            # 随机选择物体的起始位置
            x = np.random.randint(self.grid_length//4, 3*self.grid_length//4)
            y = np.random.randint(self.grid_width//4, 3*self.grid_width//4)
            
            # 随机物体大小
            width = np.random.randint(1, 2)  # 2-5米
            height = np.random.randint(1, 2)  # 1-3米
            
            # 确保物体不会超出地图边界
            x_end = min(x + width, self.grid_length - 2)
            y_end = min(y + height, self.grid_width - 2)
            
            # 添加物体
            self.map[y:y_end, x:x_end] = 1
            
    def generate_map(self):
        """生成完整的地图"""
        self.add_walls()
        self.add_wall_objects()
        self.add_center_objects()
        return self.map
    
    def visualize_map(self):
        """可视化地图"""
        plt.figure(figsize=(15, 5))
        plt.imshow(self.map, cmap='binary', extent=[0, self.length, 0, self.width])
        plt.grid(True, alpha=0.3)
        plt.title('Room Map')
        plt.colorbar(label='Occupancy (0:Free, 1:Occupied)')
        
        # 添加真实尺寸的刻度
        x_ticks = np.linspace(0, self.length, 7)
        y_ticks = np.linspace(0, self.width, 5)
        plt.xticks(x_ticks, [f'{x:.1f}m' for x in x_ticks])
        plt.yticks(y_ticks, [f'{y:.1f}m' for y in y_ticks])
        
        plt.show()
        
    def save_map(self, filename='room_map.npy'):
        """保存地图到文件，重采样为0.1m分辨率"""
        # 计算重采样后的尺寸
        new_width = int(self.width / self.final_resolution)
        new_length = int(self.length / self.final_resolution)
        
        # 使用最近邻插值进行重采样
        zoom_factor = (new_width / self.grid_width, new_length / self.grid_length)
        resampled_map = zoom(self.map, zoom_factor, order=0)  # order=0 表示最近邻插值
        
        # 创建包含坐标信息的字典
        map_data = {
            'map': resampled_map,
            'resolution': self.final_resolution,
            'origin': [0, 0],  # 设置原点为 [0, 0]
            'width': self.width,
            'length': self.length
        }
        
        # 保存重采样后的地图
        np.save(filename, map_data)
        print(f"Map saved to {filename} with resolution {self.final_resolution}m")

def main():
    # 创建地图生成器
    map_gen = MapGenerator(length=30, width=10, resolution=1.0)
    
    # 生成地图
    room_map = map_gen.generate_map()
    
    # 可视化地图
    map_gen.visualize_map()
    
    # 保存地图
    map_gen.save_map()

if __name__ == "__main__":
    main()