#import "@preview/bubble-sysu:0.1.0": *

#show: report.with(
  title: "机器人导论作业一",
  subtitle: "ROS 环境下四轮差速小车建模与基本控制实验",
  student: (name: "元朗曦", id: "23336294"),
  school: "计算机学院",
  major: "计算机科学与技术",
  class: "计八",
)

= 实验背景

机器人系统通常由感知、决策、规划和控制等模块组成，各模块之间需要稳定的数据通信和统一的运行管理。ROS 提供了节点、话题、服务、参数服务器、可视化工具和仿真工具等能力，适合用于机器人模型搭建、控制算法验证和系统联调。

本实验围绕四轮差速小车展开，目标是在 ROS Noetic 环境中完成机器人模型描述、RViz 可视化展示和 Gazebo 仿真运动控制。实验使用 URDF/Xacro 描述机器人结构，使用 RViz 检查模型和坐标关系，使用 Gazebo 作为仿真环境，并通过 ROS topic 发布键盘速度指令，驱动自写控制器控制小车运动。

= 实验内容

本实验项目根目录为 `lab/01`，主要文件包括 Docker 工具链配置、catkin 工作空间、四轮小车 ROS 功能包、模型文件、启动文件和控制脚本。

== 环境准备

实验使用 Docker 配置 ROS 工具链，基础镜像为 Docker Hub 官方镜像 `ros:noetic-ros-base-focal`。在此基础上安装 catkin、xacro、RViz、Gazebo ROS、robot_state_publisher、joint_state_publisher、VNC/noVNC 等依赖。

进入项目根目录：

```bash
cd lab/01
```

启动容器。若本地不存在镜像，Docker Compose 会根据 `Dockerfile` 自动构建镜像；容器启动时会自动进入 `/workspace/catkin_ws` 并执行 `catkin_make`，完成 ROS 工程编译：

```bash
docker compose up
```

如果需要查看图形界面，在浏览器中打开 noVNC 页面：

```text
http://localhost:6080/vnc.html
```

默认 VNC 密码为：

```text
ros
```

容器完成自动编译后会继续自动执行 `roslaunch four_wheel_diff_car bringup.launch`，因此打开 noVNC 页面后即可看到 RViz、Gazebo 和键盘控制窗口。

本实验包名为 `four_wheel_diff_car`。功能包中包含两个 Python 节点：`keyboard_teleop.py` 用于发布 `/cmd_vel` 键盘速度指令，`simple_diff_drive_controller.py` 用于订阅 `/cmd_vel` 并更新 Gazebo 中的小车位姿，同时发布 `/joint_states` 供 RViz 显示车轮状态。

== RViz 模型显示

自动启动的 `bringup.launch` 会同时打开 RViz。若 RViz 中出现蓝色车体、灰色上盖和四个黑色车轮，且左侧 `RobotModel` 状态为 `Ok`，说明 URDF/Xacro 模型、固定坐标系和关节连接关系配置正确。

项目中也保留了 `display.launch`，它只用于模型展示，不启动 Gazebo，也不启动运动控制器。

== Gazebo 仿真与键盘控制

自动启动的 `bringup.launch` 会启动 Gazebo、加载小车模型、启动自写差速控制器、启动键盘控制窗口，并同时打开 RViz。在 noVNC 桌面中找到标题为 `keyboard_teleop` 的终端窗口，点击该窗口使其获得键盘焦点。使用以下按键控制小车：

```text
w / s : 前进 / 后退
a / d : 左转 / 右转
x     : 停止
q / e : 增大 / 减小线速度
z / c : 增大 / 减小角速度
Ctrl-C: 退出键盘控制节点
```

键盘控制节点将速度指令发布到 `/cmd_vel` 话题。自写控制器订阅该话题，根据线速度和角速度积分得到小车在二维平面上的位姿，并通过 Gazebo 的模型状态接口更新仿真模型位置；同时根据左右轮速度计算轮子角度并发布 `/joint_states`，使 RViz 中车轮状态随运动同步变化。

== 启动文件区别

本实验提供三个主要启动文件：

- `display.launch`：只启动 RViz 模型展示，用于检查小车外观和关节连接。
- `gazebo.launch`：启动 Gazebo、小车模型、键盘控制节点和自写控制器。
- `bringup.launch`：同时启动 Gazebo 控制链路和 RViz，用于完整演示实验效果。

= 实验结果

实验成功完成了四轮差速小车的 ROS 建模与基本控制。运行 `display.launch` 后，RViz 能正确显示小车模型，模型包含蓝色车体、灰色上盖和四个黑色车轮，`RobotModel` 状态正常，说明机器人描述文件能够被 ROS 正确解析。

运行 `bringup.launch` 后，Gazebo 中能够生成四轮小车模型，键盘控制窗口能够向 `/cmd_vel` 话题发布速度指令。按下 `w`、`s`、`a`、`d` 等按键后，小车可在 Gazebo 中完成前进、后退、左转和右转；按下 `x` 后小车停止运动。实验表明，键盘控制节点、ROS topic 通信、自写控制器和 Gazebo 模型状态更新流程均已跑通。

编译过程中出现了 `gazebo_msgs` deprecated 的警告，这是由于 Gazebo Classic 11 已在 2025 年 1 月到达生命周期终点。由于 ROS Noetic 课程环境仍使用 Gazebo Classic，该警告不影响本实验编译和运行。

= 收获体会

通过本实验，我熟悉了 ROS 工程从环境配置到模型展示再到仿真控制的基本流程。URDF/Xacro 使机器人模型能够以结构化方式描述，RViz 可以快速检查模型、关节和坐标关系，Gazebo 则提供了更接近实际运行场景的仿真环境。

实验也加深了我对 ROS 通信机制的理解。键盘节点和控制器节点之间并不直接调用函数，而是通过 `/cmd_vel` 话题传递速度指令，这体现了 ROS 发布/订阅模型的解耦特点。控制器订阅速度指令后再更新 Gazebo 模型状态，展示了从上层控制命令到仿真执行效果的完整链路。

此外，使用 Docker 搭建 ROS Noetic 环境降低了本机系统差异带来的影响。即使在 macOS 上，也可以通过容器和 noVNC 运行 Linux 图形界面程序。这种方式提高了实验环境的可复现性，也便于后续继续开展机器人建模、控制和仿真实验。
