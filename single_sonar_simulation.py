import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable

class SingleUltrasonicSensor:
    def __init__(self, position=(0,0), beam_width=np.pi/6, max_range=5.0, max_error=0.1):
        """
        初始化超声波传感器
        
        参数:
        position (tuple): 传感器位置 (x_s, y_s)
        beam_width (float): 波束宽度 w (弧度)
        max_range (float): 最大测量范围 (米)
        max_error (float): 最大测量误差 E (米)
        """
        self.position = np.array(position)
        self.beam_width = beam_width
        self.max_range = max_range
        self.max_error = max_error
        self.R_min = 0.1  # 最小测量范围
        
    def calculate_delta_theta(self, point):
        """
        计算点P到传感器的距离(delta)和角度(theta)
        
        参数:
        point (numpy.array): 点P的坐标 (x, y)
        
        返回:
        tuple: (delta, theta)
        """
        vector = point - self.position
        delta = np.linalg.norm(vector)
        
        # ���设传感器朝向x轴正方向
        reference_vector = np.array([1, 0])
        cos_theta = np.dot(vector, reference_vector) / (delta * np.linalg.norm(reference_vector))
        theta = np.arccos(np.clip(cos_theta, -1, 1))
        
        return delta, theta
    
    def calculate_probability(self, point, R):
        """
        计算点P的概率值
        返回值范围:
        - unknown: 0
        - empty: [-1, 0)  # 将原始空置概率[0,1]映射到[-1,0)
        - occupied: (0, 1] # 保持占用概率在(0,1]范围内
        
        参数:
        point (numpy.array): 点P的坐标
        R (float): 传感器返回的测量值
        
        返回:
        float: 概率值
        """
        delta, theta = self.calculate_delta_theta(point)
        
        # 如果角度太大，返回0（未知）
        if abs(theta) > self.beam_width/2:
            return 0
            
        # 计算空置概率（距离在测量值之前）
        if delta < R - self.max_error:
            E_r = np.maximum(0, 1 - ((delta - self.R_min)/(R - self.max_error - self.R_min))**2)
            E_a = np.maximum(0, 1 - (2*theta/self.beam_width)**2)
            empty_prob = E_r * E_a
            return -empty_prob  # 将[0,1]范围的空置概率映射到[-1,0]范围
            
        # 计算占用概率��距离在测量值附近）
        if R - self.max_error <= delta <= R + self.max_error:
            O_r = np.maximum(0, 1 - ((delta - R)/self.max_error)**2)
            O_a = np.maximum(0, 1 - (2*theta/self.beam_width)**2)
            return O_r * O_a  # 保持在(0,1]范围内
            
        # 其他情况返回0（未知）
        return 0
    
    def visualize_3d(self, R, grid_resolution=0.1):
        """
        可视化3D概率分布
        """
        # 创建2D网格
        x = np.arange(-1, R + 2, grid_resolution)
        y = np.arange(-2, 2, grid_resolution)
        X, Y = np.meshgrid(x, y)
        
        # 计算每个点的概率
        Z = np.zeros_like(X)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                point = np.array([X[i,j], Y[i,j]])
                Z[i,j] = self.calculate_probability(point, R)
        
        # 创建3D图
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制空置概率（蓝色）
        empty_mask = Z < 0
        if np.any(empty_mask):
            surf1 = ax.plot_surface(X, Y, np.where(empty_mask, Z, np.nan),
                                   cmap='Blues', alpha=0.6)
        
        # 绘制占用概率（红色）
        occupied_mask = Z > 0
        if np.any(occupied_mask):
            surf2 = ax.plot_surface(X, Y, np.where(occupied_mask, Z, np.nan),
                                   cmap='Reds', alpha=0.6)
        
        # 绘制未知区域（灰色）
        unknown_mask = Z == 0
        if np.any(unknown_mask):
            surf3 = ax.plot_surface(X, Y, np.where(unknown_mask, Z, np.nan),
                                   color='gray', alpha=0.3)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Probability')
        ax.set_zlim(-1, 1)
        ax.view_init(elev=1, azim=-1)
        ax.dist = 10
        ax.set_title('Probability Distribution')

        # 添加空置概率和占用概率的颜色条
        empty_norm = plt.Normalize(-1, 0)
        occ_norm = plt.Normalize(0, 1)
        
        # 直接在图形上添加颜色条
        fig.colorbar(plt.cm.ScalarMappable(norm=empty_norm, cmap='Blues'), 
                    ax=ax, label='Empty Probability', pad=0.1)
        fig.colorbar(plt.cm.ScalarMappable(norm=occ_norm, cmap='Reds'), 
                    ax=ax, label='Occupancy Probability', pad=0.15)
        
        plt.show()
        
        return X, Y, Z  # 返回网格数据供2D图使用
    
    def visualize_2d_sections(self, X, Y, Z, R, save_path='cross_sections.png'):
        """
        绘制并保存2D切面图
        """
        fig = plt.figure(figsize=(15, 12))
        
        # y=0处的切面图（左上）
        ax1 = fig.add_subplot(221)
        y_idx = np.abs(Y[:,0]).argmin()
        ax1.plot(X[0,:], Z[y_idx, :], 'k-', label='y=0')
        ax1.grid(True)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Probability')
        ax1.set_ylim(-1, 1)
        ax1.set_title('Cross Section at y=0')
        ax1.legend()

        # x=1,2,3处的切面图（右上）
        ax2 = fig.add_subplot(222)
        x_values = [1, 2, 3]
        for x_val in x_values:
            x_idx = np.abs(X[0,:] - x_val).argmin()
            ax2.plot(Y[:,0], Z[:, x_idx], label=f'x={x_val}')
        ax2.grid(True)
        ax2.set_xlabel('Y (m)')
        ax2.set_ylabel('Probability')
        ax2.set_ylim(-1, 1)
        ax2.set_title('Cross Sections at x=1,2,3')
        ax2.legend()

        # 等高线图（左下）
        ax3 = fig.add_subplot(223)
        contour = ax3.contour(X, Y, Z, levels=np.linspace(-1, 1, 21), cmap='BuRd')
        ax3.clabel(contour, inline=True, fontsize=8)
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Y (m)')
        ax3.set_title('Contour Plot')
        ax3.grid(True)

        plt.tight_layout()
        plt.savefig(save_path)
        plt.close()

def main():
    # 创建传感器实例
    sensor = SingleUltrasonicSensor(
        position=(0,0),
        beam_width=np.pi/6,  # 30度
        max_range=5.0,
        max_error=0.1
    )
    
    # 模拟一次测量并可视化
    R = 3.0  # 假设测量到障碍物距离为3米
    
    # 先显示3D图并获取网格数据
    X, Y, Z = sensor.visualize_3d(R, grid_resolution=0.1)
    
    # 保存2D切面图
    sensor.visualize_2d_sections(X, Y, Z, R, save_path='sonar_cross_sections.png')

if __name__ == "__main__":
    main() 