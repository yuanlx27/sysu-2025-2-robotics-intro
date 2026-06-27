## Docker ROS 环境

进入作业二目录：

```bash
cd lab/02
```

构建 ROS Noetic 镜像：

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

容器的 root shell 会自动加载 ROS 环境；如果已经编译过工作空间，也会自动加载 `/workspace/catkin_ws/devel/setup.bash`。

如果需要使用 RViz 或 RQT，在浏览器中打开 noVNC 桌面：

```text
http://localhost:6081/vnc.html
```

默认 VNC 密码：

```text
ros
```

## 编译与运行

在容器内执行：

```bash
cd /workspace/catkin_ws
catkin_make
```

加载工作空间环境：

```bash
source devel/setup.bash
```

运行轨迹跟踪实验：

```bash
roslaunch tracking_pid test_tracking_pid.test rviz:=true
```

打开 RQT 动态调参界面：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

可选：运行测试：

```bash
catkin_make run_tests
```
