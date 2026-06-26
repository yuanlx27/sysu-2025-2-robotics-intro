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
    ros-noetic-catkin \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    ros-noetic-rqt-graph \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-xacro \
    tigervnc-common \
    tigervnc-standalone-server \
    websockify \
    xfce4 \
    xfce4-terminal \
    xterm \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

WORKDIR /workspace

COPY scripts/entrypoint.sh /ros_entrypoint_lab01.sh
RUN chmod +x /ros_entrypoint_lab01.sh

EXPOSE 6080

ENTRYPOINT ["/ros_entrypoint_lab01.sh"]
CMD ["bash", "-lc", "tail -f /dev/null"]
