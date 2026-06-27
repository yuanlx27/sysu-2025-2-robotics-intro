#!/usr/bin/env python
import rospy
import numpy as np
import math
import random
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

OCCUPIED_THRESHOLD = 50
ROBOT_RADIUS = 0.2
STEP_SIZE = 0.5
GOAL_THRESHOLD = 0.5
MAX_ITER = 5000

def calc_distance(p_1, p_2):
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def point_to_line(p_1, p_2, p_3):
    dist = math.sqrt((p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2)

    # determine intersection ratio u
    # for three points A, B with a line between them and a third point C, the tangent to the line AB
    # passing through C intersects the line AB a distance along its length equal to u*|AB|
    r_u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(dist**2)

    # intersection point
    p_i = (p_1[0] + r_u*(p_2[0] - p_1[0]), p_1[1] + r_u*(p_2[1] - p_1[1]))

    # distance from P3 to intersection point
    tan_len = calc_distance(p_i, p_3)

    return r_u, tan_len

class MapProcessor:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.obstacles = []
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.extract_obstacles()
        rospy.loginfo("Map loaded with resolution %.3f at origin (%.2f, %.2f)" % (
            self.map_info.resolution, 
            self.map_info.origin.position.x,
            self.map_info.origin.position.y))

    def extract_obstacles(self):
        self.obstacles = []
        for y in range(self.map_info.height):
            for x in range(self.map_info.width):
                if self.map_data[y][x] > 50:  # 障碍物阈值
                    world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                    world_y = y * self.map_info.resolution + self.map_info.origin.position.y
                    self.obstacles.append((world_x, world_y))

    def world_to_map(self, point):
        x = int(math.floor((point.x - self.map_info.origin.position.x) / self.map_info.resolution))
        y = int(math.floor((point.y - self.map_info.origin.position.y) / self.map_info.resolution))
        return (x, y)

    def is_collision(self, p1, p2):
        if self.map_info is None or self.map_data is None:
            return True

        resolution = self.map_info.resolution
        distance = math.hypot(p2.x - p1.x, p2.y - p1.y)
        steps = max(1, int(math.ceil(distance / (resolution * 0.5))))

        for i in range(steps + 1):
            ratio = float(i) / steps
            point = Point(
                p1.x + ratio * (p2.x - p1.x),
                p1.y + ratio * (p2.y - p1.y),
                0
            )
            if self.is_occupied(point):
                return True
        return False

    def is_occupied(self, point):
        mx, my = self.world_to_map(point)
        if not self.is_in_map(mx, my):
            return True

        inflation_cells = int(math.ceil(ROBOT_RADIUS / self.map_info.resolution))
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if math.hypot(dx, dy) * self.map_info.resolution > ROBOT_RADIUS:
                    continue

                cx = mx + dx
                cy = my + dy
                if not self.is_in_map(cx, cy):
                    return True

                value = self.map_data[cy][cx]
                if value < 0 or value > OCCUPIED_THRESHOLD:
                    return True
        return False

    def is_in_map(self, x, y):
        return 0 <= x < self.map_info.width and 0 <= y < self.map_info.height

class RRTPlanner:
    def __init__(self):
        self.mp = MapProcessor()
        self.start = Point(
            rospy.get_param('~start_x', 0.5),
            rospy.get_param('~start_y', 19.5),
            0
        )
        self.goal = None
        self.nodes = []
        
        # 双路径发布器
        self.vis_path_pub = rospy.Publisher('/rrt_path', Path, queue_size=10)  # 可视化用
        self.ctrl_path_pub = rospy.Publisher('/path', Float32MultiArray, queue_size=10)  # 控制用
        self.tree_pub = rospy.Publisher('/rrt_tree', MarkerArray, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.wait_for_message('/map', OccupancyGrid)
        rospy.loginfo("Planner initialized, waiting for goal...")

    class Node:
        def __init__(self, point, parent=None):
            self.point = point
            self.parent = parent
            self.cost = 0.0 if parent is None else parent.cost + math.hypot(
                point.x - parent.point.x, point.y - parent.point.y)

    def goal_callback(self, msg):
        self.goal = msg.pose.position
        if self.mp.map_info is not None:
            self.plan_path()

    def plan_path(self):
        if self.start is None or self.goal is None:
            rospy.logwarn("Start or goal is not set.")
            return

        if self.mp.is_occupied(self.start):
            rospy.logwarn("Start point is outside the map or in collision.")
            return

        if self.mp.is_occupied(self.goal):
            rospy.logwarn("Goal point is outside the map or in collision.")
            return

        self.nodes = [self.Node(self.start)]
        rospy.loginfo("Starting basic RRT planning from (%.2f, %.2f) to (%.2f, %.2f)" % (
            self.start.x, self.start.y, self.goal.x, self.goal.y))

        for iteration in range(MAX_ITER):
            random_point = self.sample_free_point()
            nearest_node = self.get_nearest_node(random_point)
            new_point = self.steer(nearest_node.point, random_point)

            if self.mp.is_collision(nearest_node.point, new_point):
                continue

            new_node = self.Node(new_point, nearest_node)
            self.nodes.append(new_node)

            if self.distance_between_points(new_point, self.goal) <= GOAL_THRESHOLD:
                if not self.mp.is_collision(new_point, self.goal):
                    goal_node = self.Node(Point(self.goal.x, self.goal.y, 0), new_node)
                    self.nodes.append(goal_node)
                    rospy.loginfo("RRT reached goal after %d iterations with %d nodes." % (
                        iteration + 1, len(self.nodes)))
                    self.publish_path(goal_node)
                    return

            if (iteration + 1) % 200 == 0:
                self.publish_tree()

        self.publish_tree()
        rospy.logwarn("RRT failed to find a collision-free path after %d iterations." % MAX_ITER)

    def sample_free_point(self):
        origin = self.mp.map_info.origin.position
        width = self.mp.map_info.width * self.mp.map_info.resolution
        height = self.mp.map_info.height * self.mp.map_info.resolution

        for _ in range(100):
            point = Point(
                random.uniform(origin.x, origin.x + width),
                random.uniform(origin.y, origin.y + height),
                0
            )
            if not self.mp.is_occupied(point):
                return point

        return Point(
            random.uniform(origin.x, origin.x + width),
            random.uniform(origin.y, origin.y + height),
            0
        )

    def get_nearest_node(self, point):
        return min(self.nodes, key=lambda node: self.distance_between_points(node.point, point))

    def steer(self, from_point, to_point):
        distance = self.distance_between_points(from_point, to_point)
        if distance <= STEP_SIZE:
            return Point(to_point.x, to_point.y, 0)

        theta = math.atan2(to_point.y - from_point.y, to_point.x - from_point.x)
        return Point(
            from_point.x + STEP_SIZE * math.cos(theta),
            from_point.y + STEP_SIZE * math.sin(theta),
            0
        )

    def distance_between_points(self, p1, p2):
        return math.hypot(p2.x - p1.x, p2.y - p1.y)


    def publish_path(self, goal_node):
        # 可视化Path构建
        vis_path = Path()
        vis_path.header.frame_id = "map"
        vis_path.header.stamp = rospy.Time.now()
        
        # 控制用路径数据构建
        ctrl_path = Float32MultiArray()
        path_points = []
        
        # 收集路径点（从终点到起点）
        current = goal_node
        while current is not None:
            # 可视化Path的点
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position = current.point
            vis_path.poses.append(pose)
            
            # 控制用路径点
            path_points.append((current.point.x, current.point.y))
            current = current.parent
        
        # 反转路径顺序（起点到终点）
        vis_path.poses.reverse()
        path_points.reverse()
        
        # 填充控制路径数据
        for point in path_points:
            ctrl_path.data.extend([point[0], point[1]])
        
        # 同时发布两种格式
        self.vis_path_pub.publish(vis_path)
        self.ctrl_path_pub.publish(ctrl_path)
        rospy.loginfo("Published both path formats:\n" +
                     f"- Visual Path ({len(vis_path.poses)} poses)\n" +
                     f"- Control Path ({len(path_points)} points)")
        
        self.publish_tree()

    def publish_tree(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        for node in self.nodes:
            if node.parent:
                marker.points.append(node.parent.point)
                marker.points.append(node.point)
        
        marker_array.markers.append(marker)
        self.tree_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('rrt_planner')
    planner = RRTPlanner()
    rospy.spin()
