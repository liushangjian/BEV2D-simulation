import numpy as np

# 加载地图文件
map_path = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\files\room_map.npy"
map_data = np.load(map_path, allow_pickle=True).item()

# 打印地图信息
print("Map information:")
print(f"Resolution: {map_data['resolution']}m")
print(f"Origin: {map_data['origin']}")
print(f"Width: {map_data['width']}m")
print(f"Length: {map_data['length']}m")
print(f"Map shape: {map_data['map'].shape}")

# 可选：显示地图
import matplotlib.pyplot as plt

plt.figure(figsize=(15, 5))
plt.imshow(map_data['map'], origin='lower', cmap='gray',
          extent=[0, map_data['length'], 0, map_data['width']])
plt.colorbar(label='Occupancy')
plt.title('Room Map')
plt.grid(True)
plt.show() 