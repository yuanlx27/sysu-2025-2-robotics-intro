#!/usr/bin/env python3
import rospy
import random
import numpy as np
from PIL import Image
import yaml
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

# ================== 配置参数 ==================
MAP_WIDTH = 20.0    # 地图宽度（米）
MAP_HEIGHT = 20.0   # 地图高度（米）
RESOLUTION = 0.1     # 地图分辨率（米/像素）
CELL_SIZE = 1.0      # 课程设计基础地图每个栅格对应 1 米
OCCUPANCY_THRESHOLD = 0.65  # 占据阈值
ppgm=rospy.get_param('pgm_path')
pyaml=rospy.get_param('yaml_path')

BASIC_GRID = [
    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,0,0,1,1,1,0,1,1,1,1,0,0,0,0,0,0],
    [0,1,1,1,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0],
    [1,1,1,1,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0],
    [1,1,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0],
]

# ================== 障碍物生成模块 ==================
class ObstacleGenerator:
    def __init__(self):
        self.obstacles = []  # 存储障碍物信息 (x, y, size_x, size_y)
        
    def generate_in_gazebo(self, num_obstacles=10):
        """在 Gazebo 中生成障碍物并记录参数"""
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        for i in range(num_obstacles):
            # 随机生成障碍物参数
            x = random.uniform(0, MAP_WIDTH)
            y = random.uniform(0, MAP_HEIGHT)
            size_x = random.uniform(0.5, 3.0)
            size_y = random.uniform(0.5, 3.0)
            size_z = random.uniform(0.5, 2.0)
            half_x = size_x / 2
            half_y = size_y / 2
            x_min_check=x - half_x
            y_min_check=y - half_y
            x_max_check=x + half_x
            y_max_check=y + half_y
            safe_points = [(0.5, MAP_HEIGHT - 0.5), (MAP_WIDTH - 0.5, 0.5)]
            near_threshold = 1.5

            blocks_safe_point = any(
                (x_min_check <= sx <= x_max_check and y_min_check <= sy <= y_max_check)
                or (abs(x - sx) < half_x + near_threshold and abs(y - sy) < half_y + near_threshold)
                for sx, sy in safe_points
            )
            if blocks_safe_point:
              continue
            self.obstacles.append((x, y, size_x, size_y))

            # 生成 Gazebo 模型
            model_name = f"obstacle_{i}"
            model_xml = self._create_sdf_model(size_x, size_y, size_z)
            pose = Pose(position=Point(x, y, size_z/2), orientation=Quaternion(0,0,0,1))
            
            try:
                req = SpawnModelRequest()
                req.model_name = model_name
                req.model_xml = model_xml
                req.initial_pose = pose
                req.reference_frame = "world"
                spawn_model(req)
                rospy.loginfo(f"Spawned {model_name} at ({x:.2f}, {y:.2f})")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn model: {e}")

    def generate_basic_grid_in_gazebo(self):
        """按照课程给定的 20x20 01 矩阵生成 Gazebo 障碍物。"""
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        rows = len(BASIC_GRID)
        for row_idx, row in enumerate(BASIC_GRID):
            for col_idx, cell in enumerate(row):
                if cell != 1:
                    continue

                x = (col_idx + 0.5) * CELL_SIZE
                y = (rows - row_idx - 0.5) * CELL_SIZE
                size_z = 0.8
                self.obstacles.append((x, y, CELL_SIZE, CELL_SIZE))

                model_name = f"grid_obstacle_{row_idx}_{col_idx}"
                model_xml = self._create_sdf_model(CELL_SIZE, CELL_SIZE, size_z)
                pose = Pose(position=Point(x, y, size_z / 2.0), orientation=Quaternion(0, 0, 0, 1))

                try:
                    req = SpawnModelRequest()
                    req.model_name = model_name
                    req.model_xml = model_xml
                    req.initial_pose = pose
                    req.reference_frame = "world"
                    spawn_model(req)
                    rospy.loginfo(f"Spawned {model_name} at ({x:.2f}, {y:.2f})")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to spawn model: {e}")

    def _create_sdf_model(self, size_x, size_y, size_z):
        """生成障碍物 SDF 模型"""
        return f"""
        <sdf version="1.6">
          <model>
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{size_x} {size_y} {size_z}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{size_x} {size_y} {size_z}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{random.random()} {random.random()} {random.random()} 1</ambient>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

# ================== PGM 地图生成模块 ==================
class PGMMapGenerator:
    def __init__(self):
        self.grid = np.ones((
            int(MAP_HEIGHT / RESOLUTION), 
            int(MAP_WIDTH / RESOLUTION)
        ), dtype=np.uint8) * 255

    def world_to_grid(self, x, y):
        """世界坐标转像素坐标"""
        grid_x = int(x / RESOLUTION)
        grid_y = int((MAP_HEIGHT - y) / RESOLUTION)
        return grid_x, grid_y

    def add_obstacle(self, x, y, size_x, size_y):
        
        """添加矩形障碍物到地图"""
        # 计算障碍物覆盖的像素范围
        half_x = size_x / 2
        half_y = size_y / 2
        x_min, y_min = self.world_to_grid(x - half_x, y - half_y)
        x_max, y_max = self.world_to_grid(x + half_x, y + half_y)

        # 确保索引不越界
        x_min = max(0, min(self.grid.shape[1], x_min))
        x_max = max(0, min(self.grid.shape[1], x_max))
        y_min = max(0, min(self.grid.shape[0], y_min))
        y_max = max(0, min(self.grid.shape[0], y_max))
        if x_min > x_max:
            x_min, x_max = x_max, x_min
        if y_min > y_max:
            y_min, y_max = y_max, y_min
        self.grid[y_min:y_max, x_min:x_max] = 0  # 黑色代表障碍物

    def load_basic_grid(self):
        cells_per_grid = int(CELL_SIZE / RESOLUTION)
        rows = len(BASIC_GRID)
        cols = len(BASIC_GRID[0])
        self.grid = np.ones((rows * cells_per_grid, cols * cells_per_grid), dtype=np.uint8) * 255

        for row_idx, row in enumerate(BASIC_GRID):
            for col_idx, cell in enumerate(row):
                if cell == 1:
                    y0 = row_idx * cells_per_grid
                    y1 = y0 + cells_per_grid
                    x0 = col_idx * cells_per_grid
                    x1 = x0 + cells_per_grid
                    self.grid[y0:y1, x0:x1] = 0

    def save(self, pgm_path=ppgm, yaml_path=pyaml):
  
        """保存PGM和YAML文件"""
        Image.fromarray(self.grid).save(pgm_path)
        
        # 生成YAML元数据
        yaml_data = {
            "image": pgm_path,
            "resolution": RESOLUTION,
            "origin": [0.0, 0.0, 0.0],
            "occupied_thresh": OCCUPANCY_THRESHOLD,
            "free_thresh": 0.25,
            "negate": 0
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
     

# ================== 主流程 ==================
if __name__ == "__main__":
    rospy.init_node('spawn_obstacles')
    map_mode = rospy.get_param('~map_mode', 'basic')
    obstacle_count = rospy.get_param('~obstacle_count', 15)

    obstacle_gen = ObstacleGenerator()
    map_gen = PGMMapGenerator()

    if map_mode == 'random':
        obstacle_gen.generate_in_gazebo(num_obstacles=obstacle_count)
        for obs in obstacle_gen.obstacles:
            x, y, size_x, size_y = obs
            map_gen.add_obstacle(x, y, size_x, size_y)
        print(ppgm)
        print(pyaml)
        map_gen.save()
    else:
        map_gen.load_basic_grid()
        print(ppgm)
        print(pyaml)
        map_gen.save()
        obstacle_gen.generate_basic_grid_in_gazebo()

    print("PGM地图已生成: gazebo_map.pgm 和 gazebo_map.yaml")
