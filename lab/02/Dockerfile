FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    dbus-x11 \
    git \
    mesa-utils \
    novnc \
    python3-catkin-tools \
    python3-numpy \
    python3-pip \
    python3-rosdep \
    python3-rostest \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-angles \
    ros-noetic-catkin \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-geometry-msgs \
    ros-noetic-map-server \
    ros-noetic-nav-core \
    ros-noetic-nav-msgs \
    ros-noetic-pluginlib \
    ros-noetic-roscpp \
    ros-noetic-roslint \
    ros-noetic-rosunit \
    ros-noetic-rqt-reconfigure \
    ros-noetic-rviz \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-std-srvs \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-visualization-msgs \
    tigervnc-common \
    tigervnc-standalone-server \
    websockify \
    xfce4 \
    xfce4-terminal \
    xterm \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

WORKDIR /workspace

COPY scripts/entrypoint.sh /ros_entrypoint_lab02.sh
RUN chmod +x /ros_entrypoint_lab02.sh

EXPOSE 6080

ENTRYPOINT ["/ros_entrypoint_lab02.sh"]
