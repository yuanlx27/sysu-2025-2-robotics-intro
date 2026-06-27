#import "@preview/bubble-sysu:0.1.0": *

#show: report.with(
  title: "机器人导论作业二",
  subtitle: "PID 轨迹跟踪控制实验",
  student: (name: "元朗曦", id: "23336294"),
  school: "计算机学院",
  major: "计算机科学与技术",
  class: "计八",
)

= 实验介绍

本实验围绕 ROS 环境下的移动机器人轨迹跟踪控制展开，目标是在给定仿真代码包中补全 PID 控制器，使机器人能够跟随预设轨迹运动。实验使用 ROS 的 topic、TF、dynamic_reconfigure、RViz 和 RQT 等工具，将路径插值节点产生的局部目标点转换为速度控制指令，并通过仿真器观察轨迹跟踪效果。

PID 控制器由比例、积分和微分三个环节组成。比例项根据当前误差快速产生控制作用，积分项用于补偿长期累积误差，微分项根据误差变化趋势抑制超调和振荡。本实验中的控制器包含纵向、横向和偏航三个控制回路，分别对应机器人前进方向误差、侧向误差和角度误差。

= 实验步骤

== 代码包架构

作业二代码位于 `lab/02`，该目录是一个 catkin 工作空间，主要包含两个 ROS 功能包：

- `tracking_pid`：轨迹插值与 PID 跟踪控制包。核心文件包括 `src/controller.cpp`、`src/controller_node.cpp`、`nodes/path_interpolator`、`cfg/Pid.cfg` 和自定义消息 `msg/PidDebug.msg`、`msg/traj_point.msg`。
- `mobile_robot_simulator`：移动机器人简易仿真器，订阅 `/cmd_vel` 或重映射后的速度指令，根据速度积分发布机器人位姿、里程计和 TF。

整体运行流程如下：

```text
nav_msgs/Path
  -> path_interpolator
  -> tracking_pid/traj_point
  -> controller
  -> geometry_msgs/Twist
  -> mobile_robot_simulator
  -> odom / TF / RViz
```

其中 `path_interpolator` 将全局路径按设定速度插值成连续目标点，发布到 `trajectory` 话题；`controller_node.cpp` 订阅目标点和当前机器人 TF，调用 `Controller::update()` 计算速度指令；`mobile_robot_simulator` 根据速度指令更新机器人状态。

== 编译与运行

进入作业二目录：

```bash
cd lab/02
```

构建 ROS Noetic Docker 镜像：

```bash
docker compose build
```

启动容器：

```bash
docker compose up -d
```

进入容器：

```bash
docker compose exec ros bash
```

若需要使用 RViz 或 RQT 图形界面，在浏览器中打开 noVNC：

```text
http://localhost:6081/vnc.html
```

默认 VNC 密码为 `ros`。

编译 catkin 工作空间：

```bash
cd /workspace/catkin_ws
catkin_make
```

加载环境：

```bash
source devel/setup.bash
```

运行系统测试和可视化界面：

```bash
roslaunch tracking_pid test_tracking_pid.test rviz:=true
```

打开 RQT 动态调参界面：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

在 RQT 中调整 `controller` 节点的 `Kp_long`、`Ki_long`、`Kd_long`、`Kp_lat`、`Ki_lat`、`Kd_lat`、`Kp_ang`、`Ki_ang`、`Kd_ang` 等参数，观察 RViz 中机器人轨迹、目标点和误差变化。

== 消息收发

`tracking_pid/msg/traj_point.msg` 中包含当前轨迹点的位姿、速度和加速度信息。本次控制器保持原始接口不变，`Controller::update()` 接收当前机器人位姿、目标位姿和采样时间，并输出 `geometry_msgs/Twist` 速度指令。

控制器节点主要发布和订阅以下内容：

- 订阅 `local_trajectory`：接收当前目标轨迹点。
- 查询 TF：获取 `map` 到 `base_link` 的机器人当前位姿。
- 发布 `move_base/cmd_vel`：输出线速度和角速度控制指令。
- 发布 `debug`：输出误差、比例项、积分项、微分项和前馈项，便于调参分析。

== 核心 PID 控制代码

在 `tracking_pid/src/controller.cpp` 中，控制器首先将当前位姿与目标位姿转换到控制点坐标系，计算纵向误差 `error_long`、横向误差 `error_lat` 和偏航误差 `error_ang`。随后对误差进行积分，使用 `windup_limit` 限制积分累积，并通过二阶低通滤波得到平滑后的误差和微分误差。

补全的核心 PID 计算逻辑如下：

```cpp
proportional_long = Kp_long * filtered_error_long.at(0);
integral_long = Ki_long * error_integral_long;
derivative_long = Kd_long * filtered_error_deriv_long.at(0);

proportional_lat = Kp_lat * filtered_error_lat.at(0);
integral_lat = Ki_lat * error_integral_lat;
derivative_lat = Kd_lat * filtered_error_deriv_lat.at(0);

proportional_ang = Kp_ang * filtered_error_ang.at(0);
integral_ang = Ki_ang * error_integral_ang;
derivative_ang = Kd_ang * filtered_error_deriv_ang.at(0);
```

三个回路的控制输出分别由对应的比例、积分和微分项组成。代码根据 `feedback_long`、`feedback_lat`、`feedback_ang` 开关决定是否启用反馈控制，根据 `feedforward_long`、`feedforward_lat`、`feedforward_ang` 开关决定是否叠加前馈项。由于当前节点接口没有把 `traj_point.velocity` 传入 `Controller::update()`，本次实验中的前馈项保持为 0，主要验证 PID 反馈控制效果。

最终控制输出经过限幅处理：

- 纵向和横向输出限制在 `lower_limit` 到 `upper_limit`。
- 偏航角速度限制在 `ang_lower_limit` 到 `ang_upper_limit`。

对非全向机器人，最终只输出 `linear.x` 和 `angular.z`；横向误差主要通过偏航控制间接修正。

= 实验结果

补全 PID 控制逻辑后，控制器能够根据目标轨迹点和当前机器人位姿持续输出速度指令。运行 `test_tracking_pid.test` 时，插值器发布的目标点沿预设路径移动，控制器根据纵向、横向和偏航误差调整机器人运动，仿真器将 `/move_base/cmd_vel` 积分为机器人位姿，RViz 中可观察到机器人逐步贴近并跟随目标轨迹。

本实验推荐的初始参数如下：

#table(
  columns: 4,
  inset: 6pt,
  align: center,
  [控制回路], [Kp], [Ki], [Kd],
  [纵向 Longitudinal], [1.0-2.0], [0.0], [0.0-0.5],
  [横向 Lateral], [1.0-4.0], [0.0], [0.3],
  [偏航 Angular], [1.0], [0.0], [0.3],
)

实际调参时，先增大比例项使机器人能够快速接近轨迹；如果出现明显振荡或超调，则适当增大微分项；若存在长期静态误差，再少量引入积分项。由于仿真轨迹跟踪主要是连续运动任务，积分项过大容易造成累积误差释放和震荡，因此本实验中优先使用 `Ki = 0`。

= 实验分析

纵向回路主要影响机器人沿自身前进方向接近目标点的速度。`Kp_long` 过小时，机器人接近目标较慢；过大时，速度输出变化剧烈，容易出现冲过目标点的现象。`Kd_long` 可以抑制接近目标时的超调，但过大也会让控制输出变得保守。

横向回路用于修正机器人相对轨迹的侧向偏差。对于非全向机器人，横向速度不会直接输出到 `linear.y`，而是通过角速度控制使车头朝向轨迹方向，从而间接减小横向误差。因此横向环和偏航环需要配合调节。若横向比例过大，机器人会频繁左右摆动；若偏航微分不足，则转向时容易出现振荡。

偏航回路控制机器人朝向目标点的角速度。适当的 `Kp_ang` 能让机器人快速转向目标方向，`Kd_ang` 则用于减小角度误差变化过快导致的超调。代码中还保留了角速度与纵向速度耦合逻辑，当启用 `coupling_ang_long` 后，较大的偏航误差会降低前进速度，使机器人先调整朝向再前进，从而提高复杂路径上的稳定性。

积分项虽然可以消除静态误差，但在轨迹跟踪任务中目标点持续移动，积分误差如果累积过多，可能在误差方向变化后继续推动机器人向旧方向运动。因此代码使用 `windup_limit` 对积分项进行抗饱和限制，实际调参时也应谨慎增大 `Ki`。

= 实验总结

通过本实验，我进一步理解了 ROS 中控制节点的组织方式，以及轨迹、TF、速度指令和仿真器之间的协作关系。轨迹跟踪并不是直接让机器人跳到目标点，而是需要根据当前误差持续计算控制量，使机器人在动态过程中逐步逼近轨迹。

补全 PID 控制器的过程也加深了我对三项控制作用的理解。比例项决定基本响应速度，积分项用于处理长期偏差，微分项用于抑制误差变化过快带来的超调。三者需要结合机器人运动模型和仿真效果进行调节，不能只追求某一个参数变大。

后续如果继续改进，可以将 `traj_point.velocity` 传入控制器，补全真正的速度前馈；也可以增加更系统的误差统计，例如均方根误差、最大横向误差和轨迹完成时间，从而更客观地比较不同 PID 参数组合的效果。

= 参考文献

1. ROS Wiki: `http://wiki.ros.org/`
2. dynamic_reconfigure: `http://wiki.ros.org/dynamic_reconfigure`
3. geometry_msgs/Twist: `http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html`
4. nobleo/tracking_pid: `https://github.com/nobleo/tracking_pid`
5. 机器人导论作业二要求 PDF 与课程课件。
