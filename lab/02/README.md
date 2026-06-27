## Docker ROS 环境

从零运行实验只需要进入作业二目录：

```bash
cd lab/02
```

启动实验环境：

```bash
docker compose up
```

如果本地还没有镜像，Docker Compose 会自动构建。容器启动后会自动完成以下步骤：

- 加载 ROS Noetic 环境
- 编译 `/workspace/catkin_ws`
- 加载 `/workspace/catkin_ws/devel/setup.bash`
- 启动 noVNC 桌面
- 启动轨迹跟踪仿真实验和 RViz

在浏览器中打开 noVNC 桌面：

```text
http://localhost:6081/vnc.html
```

默认 VNC 密码：

```text
ros
```

RViz 打开后可以观察机器人跟随目标轨迹运动。

如需打开 RQT 动态调参界面，另开终端执行：

```bash
cd lab/02
docker compose exec ros bash
rosrun rqt_reconfigure rqt_reconfigure
```

停止实验：

```bash
Ctrl-C
```
