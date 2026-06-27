#import "@preview/bubble-sysu:0.1.0": *

#show: report.with(
  title: "机器人导论作业三",
  subtitle: "基于占据栅格地图的 RRT 路径规划实验",
  student: (name: "元朗曦", id: "23336294"),
  school: "计算机学院",
  major: "计算机科学与技术",
  class: "计八",
)

= 实验背景

移动机器人自主导航通常包含地图输入、路径规划、路径跟踪和执行评估等环节。其中路径规划负责在已知地图中根据起点和终点生成一条无碰撞路径，是后续控制器能够跟踪目标轨迹的前提。

本实验围绕二维占据栅格地图上的 RRT 路径规划展开。RRT（Rapidly-exploring Random Tree，快速扩展随机树）通过在自由空间中随机采样，并不断从已有树节点向采样点扩展，逐步探索地图空间。当随机树扩展到目标点附近时，算法沿父节点回溯得到从起点到终点的路径。本次实现的是课堂讲过的基础 RRT 算法，不包含 `RRT*` 中的 ChooseParent、Rewire 等路径优化步骤，也不重点处理 PID 路径跟踪。

= 实验内容

本实验代码位于 `lab/03`，目录中包含 Docker 环境配置、catkin 工作空间、RRT 规划节点、地图生成节点、TurtleBot3 模型与 Gazebo/RViz 仿真配置。

主要文件如下：

- `Dockerfile`：基于 `ros:noetic-ros-base-focal` 配置 ROS Noetic、Gazebo、RViz、noVNC 和 Python 依赖。
- `docker-compose.yaml`：将本实验目录挂载到容器 `/workspace`，并将 noVNC 映射到本机 `6082` 端口。
- `scripts/entrypoint.sh`：容器启动后自动编译 catkin 工作空间，并启动实验 launch 文件。
- `catkin_ws/src/Robot-Planner/scripts/planner.py`：基础 RRT 路径规划节点。
- `catkin_ws/src/random_map_generator/src/spawn_obstacles.py`：根据课程给定的 20x20 栅格矩阵生成 PGM 地图和 Gazebo 障碍物。
- `catkin_ws/src/Robot-Planner/launch/obs_world.launch`：启动 Gazebo、地图服务器、规划器、控制器和 RViz。

== 地图与任务输入

课程给定地图是一个 20x20 的 01 矩阵，其中 `0` 表示自由栅格，`1` 表示障碍物栅格。实验将每个栅格映射为 1m x 1m 的地图单元，并生成分辨率为 0.1m 的 PGM 占据栅格地图。

本实验默认使用课程基础地图验证算法：

- 起点位于左上角自由栅格中心，坐标为 `(0.5, 19.5)`。
- 终点位于右下角自由栅格中心，坐标为 `(19.5, 0.5)`。
- 障碍物根据矩阵中值为 `1` 的格子生成，同时写入 Gazebo 方块模型和 PGM 地图。

如果需要验证更复杂情况，也可以通过 `map_mode:=random` 生成随机障碍物地图。

== RRT 算法实现

规划节点订阅 `/map` 读取 `nav_msgs/OccupancyGrid` 地图，订阅 `/move_base_simple/goal` 接收 RViz 中设置的目标点，并发布三类结果：

- `/rrt_tree`：随机树可视化，类型为 `visualization_msgs/MarkerArray`。
- `/rrt_path`：规划出的路径，类型为 `nav_msgs/Path`，用于 RViz 显示。
- `/path`：扁平化路径点数组，类型为 `std_msgs/Float32MultiArray`，保留给后续控制器使用。

基础 RRT 流程如下：

```text
初始化随机树，根节点为起点
循环最多 MAX_ITER 次：
  在地图自由空间中随机采样一个点
  在已有树节点中寻找距离采样点最近的节点
  从最近节点朝采样点按固定步长扩展
  检查新边是否与障碍物碰撞
  若无碰撞，则加入新节点
  若新节点接近目标点，且到目标点连线无碰撞：
    连接目标点
    沿父节点回溯生成路径
    发布随机树和路径
```

本实验使用的主要参数如下：

#table(
  columns: 2,
  inset: 6pt,
  align: center,
  [参数], [含义],
  [`STEP_SIZE = 0.5`], [每次扩展的最大步长为 0.5m],
  [`GOAL_THRESHOLD = 0.5`], [新节点距离目标小于 0.5m 时尝试连接目标],
  [`MAX_ITER = 5000`], [最多随机扩展 5000 次],
  [`ROBOT_RADIUS = 0.2`], [碰撞检测时对障碍物做 0.2m 安全膨胀],
)

= 实验运行

进入实验三目录：

```bash
cd lab/03
```

启动 Docker 容器：

```bash
docker compose up
```

容器启动后，`entrypoint.sh` 会自动完成以下任务：

```text
source /opt/ros/noetic/setup.bash
cd /workspace/catkin_ws
catkin_make
source /workspace/catkin_ws/devel/setup.bash
roslaunch turtle obs_world.launch
```

同时，脚本会启动 noVNC 图形桌面。浏览器打开：

```text
http://localhost:6082/vnc.html
```

默认 VNC 密码为：

```text
ros
```

进入桌面后，可以看到 Gazebo 和 RViz 两个主要窗口。Gazebo 用于显示 TurtleBot3 小车和障碍物，RViz 用于显示地图、随机树和规划路径。

= 图形界面验证步骤

本实验验证的重点不是只看程序是否启动，而是要在图形界面中确认 RRT 是否真的根据地图生成了可行路径。完整操作步骤如下。

== 第一步：确认 Gazebo 场景正常

打开 noVNC 后，先观察 Gazebo 窗口。正常情况下，Gazebo 中应出现 TurtleBot3 小车和若干黑色方块障碍物。小车默认位于地图左上角附近，对应起点 `(0.5, 19.5)`；障碍物分布应与课程给定的 20x20 栅格图大致一致。

如果 Gazebo 窗口为空，或小车没有出现，需要先等待几秒，因为 launch 文件中障碍物生成、地图服务器和模型加载存在延迟。如果长时间无显示，则应查看启动终端是否有 Gazebo 模型加载或 ROS 节点报错。

== 第二步：确认 RViz 地图显示正常

切换到 RViz 窗口。正常情况下，RViz 中应显示二维栅格地图：白色区域表示可通行空间，黑色区域表示障碍物。地图范围为 20m x 20m，左上角为起点区域，右下角为终点区域。

#figure(
  image("assets/images/01-focus-rviz-window.png", width: 100%),
  caption: [在 noVNC 桌面中切换到 RViz 窗口，准备查看地图与规划结果],
)

需要重点检查：

- 地图是否出现，而不是空白画面。
- 障碍物形状是否和课程给定栅格地图一致。
- 左上角和右下角是否都是自由区域。

若 RViz 中没有地图，可以在左侧 Displays 面板检查 `Map` 显示项的话题是否为 `/map`。如果 `/map` 没有数据，说明 `map_server` 没有正确读取 `gazebo_map.yaml`，需要检查地图文件中的 `origin` 和 `image` 路径。

== 第三步：用 2D Nav Goal 设置终点

在 RViz 顶部工具栏点击 `2D Nav Goal`。随后在地图右下角自由区域点击并拖动鼠标，设置目标点和目标朝向。目标点应尽量靠近右下角格子中心，即 `(19.5, 0.5)` 附近。

#figure(
  image("assets/images/02-select-2d-nav-goal.png", width: 100%),
  caption: [在 RViz 顶部工具栏选择 `2D Nav Goal`，用于向规划器发布终点],
)

设置目标后，RViz 会向 `/move_base_simple/goal` 发布一个 `PoseStamped` 消息，RRT 规划节点收到该消息后开始规划。

如果目标点点在黑色障碍物格子内，规划器会输出警告：

```text
Goal point is outside the map or in collision.
```

此时需要重新在右下角白色自由区域设置目标点。

== 第四步：观察 RRT 随机树

目标点设置后，观察 RViz 中是否出现绿色随机树。随机树对应 `/rrt_tree` 话题，用线段表示 RRT 从起点向自由空间扩展的过程。

如果绿色树不断向地图空白区域扩展，且不穿过黑色障碍物，说明随机采样、最近节点选择、定步长扩展和碰撞检测流程正在正常工作。

如果没有出现随机树，可以检查 RViz 左侧是否启用了 MarkerArray 显示项，话题是否为 `/rrt_tree`。也可以在容器中执行：

```bash
rostopic echo /rrt_tree
```

若该话题有数据，说明规划器已经发布随机树，问题主要在 RViz 显示配置。

== 第五步：观察规划路径

当 RRT 成功连接目标点后，RViz 中应出现一条从左上角起点到右下角终点的路径。该路径对应 `/rrt_path` 话题，类型为 `nav_msgs/Path`。

#figure(
  image("assets/images/03-pick-goal-on-grid.png", width: 100%),
  caption: [在右下角自由区域设置目标点后，RViz 显示规划路径和 RRT 随机树],
)

一条有效路径应满足：

- 从左上角起点附近开始。
- 最终到达右下角目标附近。
- 路径线段不穿过黑色障碍物。
- 路径可能有折线和绕行，这是基础 RRT 的正常结果。

由于基础 RRT 只保证找到一条可行路径，并不保证最短路径，因此路径不一定平滑，也不一定最优。如果路径明显绕远，属于基础 RRT 随机采样特性的体现。

== 第六步：查看终端日志确认规划结果

在运行 `docker compose up` 的终端中，成功规划时应看到类似日志：

```text
Starting basic RRT planning from (0.50, 19.50) to (...)
RRT reached goal after ... iterations with ... nodes.
Published both path formats:
- Visual Path (... poses)
- Control Path (... points)
```

其中 `RRT reached goal` 表示规划器已经找到目标；`Visual Path` 表示发布给 RViz 的路径点数量；`Control Path` 表示发布给 `/path` 的路径点数量。

如果出现：

```text
RRT failed to find a collision-free path after 5000 iterations.
```

说明本次随机采样没有在最大迭代次数内找到路径，可以重新设置目标点，或重新启动实验再次采样。

== 第七步：检查 ROS 话题

也可以另开终端进入容器检查 ROS 话题：

```bash
docker compose exec ros bash
source /workspace/catkin_ws/devel/setup.bash
rostopic list
```

正常情况下应包含：

```text
/map
/move_base_simple/goal
/rrt_tree
/rrt_path
/path
/cmd_vel
```

设置目标点后，可以查看控制器参考路径：

```bash
rostopic echo /path
```

如果 `/path` 输出类似 `[x1, y1, x2, y2, ...]` 的数组，说明规划器已经将回溯得到的路径发布给后续控制模块。

= 实验结果

在课程给定的 20x20 基础地图中，机器人起点位于左上角自由栅格，终点位于右下角自由栅格。运行实验后，在 RViz 中通过 `2D Nav Goal` 设置右下角目标点，规划器能够从起点开始生成绿色随机树，并在成功连接目标后发布路径。

从可视化结果看，随机树主要在白色自由空间中扩展，规划路径能够绕开黑色障碍物，从左上角到达右下角目标区域。终端日志中出现 `RRT reached goal` 和 `Published both path formats`，说明基础 RRT 的采样、扩展、碰撞检测、目标连接和路径回溯流程均已跑通。

在本实验中，`/rrt_path` 用于 RViz 可视化，`/path` 用于保留给后续路径跟踪控制器。虽然本次作业重点不是 PID 控制，但路径输出格式已经为后续小车跟踪路径提供了接口。

= 实验分析

RRT 的优点是实现简单，并且适合在障碍物较多的连续空间中寻找可行路径。本实验中，算法不需要预先构造完整图结构，只需要不断采样并扩展随机树。当地图中存在狭窄通道时，只要采样次数足够，随机树就有机会穿过可通行区域并到达目标。

步长 `STEP_SIZE` 会影响规划效果。步长较大时，树扩展速度快，但更容易因为线段碰撞而被拒绝；步长较小时，路径更细致，但需要更多节点和更长规划时间。本实验采用 `0.5m` 作为折中。

碰撞检测使用地图分辨率沿线段采样，并对障碍物做 `0.2m` 安全膨胀。这样可以避免路径贴着障碍物边缘穿过，提高路径的安全性。但膨胀半径过大时，一些原本较窄的通道可能会被判定为不可通过。

基础 RRT 的不足是路径质量不稳定。由于采样具有随机性，每次运行生成的随机树和路径可能不同；同时算法只寻找第一条可行路径，不会主动优化路径长度。因此，基础 RRT 得到的路径通常存在折线较多、路径不够平滑、长度不是最短等问题。

= 实验总结

通过本实验，我完成了在 ROS/Gazebo/RViz 环境下基于占据栅格地图的基础 RRT 路径规划。实验从课程给定的 20x20 01 矩阵生成地图和障碍物，在 RViz 中设置终点，通过规划节点生成随机树和无碰撞路径，并将路径分别发布给可视化模块和后续控制模块。

本实验进一步加深了我对移动机器人导航系统分层结构的理解。地图提供环境约束，RRT 负责在自由空间中寻找可行路径，RViz 用于观察算法过程和结果，Gazebo 则提供仿真执行环境。相比单纯阅读算法流程，在图形界面中观察随机树扩展和路径生成，更直观地展示了 RRT 如何探索地图空间。

后续可以在基础 RRT 的基础上继续改进。例如，引入 `RRT*` 的父节点选择和重连机制以缩短路径；对回溯路径进行平滑处理，减少不必要的折线；自动发布默认终点，减少手动操作；进一步与 PID 控制器联调，使 TurtleBot3 能够沿规划路径稳定运动到终点。

= 参考文献

1. ROS Wiki: `http://wiki.ros.org/`
2. nav_msgs/OccupancyGrid: `http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html`
3. nav_msgs/Path: `http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html`
4. visualization_msgs/MarkerArray: `http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html`
5. 机器人导论第三次作业说明、课程 RRT 课件与课程设计参考架构图。
