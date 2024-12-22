# File: autonomous_navigation.py

## Class: NavigationRobot

**Description:** No documentation available


### Methods:

#### `__init__`

No documentation available

#### `distance_to_target`

计算到目标的距离

#### `get_valid_neighbors`

获取当前位置的二级相邻格子，步长固定为0.4m

#### `find_next_position`

使用最速接近法找到下一个最佳位置

#### `navigate`

执行导航过程

#### `save_current_state`

保存当前测量状态，分别保存概率地图和符号化地图

#### `__del__`

析构函数，确保线程池被正确关闭

#### `parallel_map_update`

No documentation available

#### `create_probability_maps`

No documentation available


---

# File: create_video.py

No classes found in this file.

# File: eight_sonars_robot_single_mapping.py

## Class: RobotWithEightSonars

**Description:** No documentation available


### Methods:

#### `__init__`

初始化带有8个超声波传感器的机器人

参数:
position (tuple): 机器人位置 (x, y)
orientation (float): 机器人朝向（弧度）

#### `normalize_angle`

将角度标准化到 -π 到 π 之间

#### `simulate_measurements`

模拟超声波传感器的测量，使用扇形检测范围

#### `set_orientation`

设置机器人的新朝向，并确保角度在正确范围内

#### `measure_environment`

模拟在给定环境中进行测量

#### `create_probability_maps`

根据测量结果创建概率地图，使用与single_sonar_simulation相同的概率分布

#### `visualize_maps`

可视化占用地图和空置地图，只显示房间范围内的部分

#### `simulate_measurements`

模拟超声波传感器的测量，并在测量到的位置添加随机噪声

参数:
real_map: 真实环境地图
resolution: 网格分辨率
noise_level (float): 随机噪声的标准差（米）

返回:
measurements: 超声波测量结果（距离列表）

#### `trace_ray`

追踪光线与地图的交点

参数:
origin: 光线起始点
direction: 光线方向
real_map: 真实环境地图
resolution: 网格分辨率

返回:
intersection: 光线与地图的交点


---

# File: linear_path_mapping.py

## Class: ProgressiveMapping

**Description:** 继承MultiPositionMapping类，添加渐进式测量和可视化功能


### Methods:

#### `combine_measurements_progressively`

渐进式进行测量并保存每次测量的结果，使用随机朝向

#### `update_maps`

更新地图，将新的观测结果整合到现有地图中

#### `save_current_state`

保存当前测量状态，分别保存概率地图和符号化地图


---

# File: linear_random_path_mapping.py

## Class: RandomPathMapping

**Description:** 继承MultiPositionMapping类，添加随机路径测量和可视化功能


### Methods:

#### `combine_measurements_progressively`

渐进式进行测量并保存每次测量的结果

#### `update_maps`

更新地图，将新的观测结果整合到现有地图中

#### `save_current_state`

保存当前测量状态，分别保存概率地图和符号化地图


---

# File: map_generator.py

## Class: MapGenerator

**Description:** No documentation available


### Methods:

#### `__init__`

No documentation available

#### `add_walls`

添加房间的墙壁

#### `add_wall_objects`

在墙边添加连续的物体，包括一些突出部分。允许重叠并合并。

#### `add_center_objects`

在房间中间添加物体

#### `generate_map`

生成完整的地图

#### `visualize_map`

可视化地图

#### `save_map`

保存地图到文件，重采样为0.1m分辨率


---

# File: multi_position_mapping.py

## Class: MultiPositionMapping

**Description:** No documentation available


### Methods:

#### `__init__`

初始化多位置建图系统

参数:
grid_resolution (float): 网格分辨率
real_map: 真实环境地图

#### `update_maps`

更新概率地图，使用概率叠加公式

参数:
occupancy_map: 新的占用概率地图
empty_map: 新的空置概率地图
robot_pos: 机器人位置

#### `combine_measurements`

进行多位置测量和融合

参数:
robot_positions (list): 机器人位置列表 [(x1,y1), (x2,y2), ...]

#### `visualize_results`

可视化融合后的结果

#### `calculate_orientation`

计算机器人朝向

参数:
previous_position (tuple): 上一个测量点 (x, y)
current_position (tuple): 当前测量点 (x, y)

返回:
float: 机器人朝向（弧度）


---

# File: single_sonar_simulation.py

## Class: SingleUltrasonicSensor

**Description:** No documentation available


### Methods:

#### `__init__`

初始化超声波传感器

参数:
position (tuple): 传感器位置 (x_s, y_s)
beam_width (float): 波束宽度 w (弧度)
max_range (float): 最大测量范围 (米)
max_error (float): 最大测量误差 E (米)

#### `calculate_delta_theta`

计算点P到传感器的距离(delta)和角度(theta)

参数:
point (numpy.array): 点P的坐标 (x, y)

返回:
tuple: (delta, theta)

#### `calculate_probability`

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

#### `visualize_3d`

可视化3D概率分布

#### `visualize_2d_sections`

绘制并保存2D切面图


---

# File: view_map.py

No classes found in this file.
