FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    dbus-x11 \
    git \
    mesa-utils \
    novnc \
    python3-catkin-tools \
    python3-numpy \
    python3-pil \
    python3-pip \
    python3-rosdep \
    python3-rostest \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-diagnostic-msgs \
    ros-noetic-diagnostic-updater \
    ros-noetic-gazebo-msgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-geometry-msgs \
    ros-noetic-interactive-markers \
    ros-noetic-map-server \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-nav-msgs \
    ros-noetic-robot-state-publisher \
    ros-noetic-rospy \
    ros-noetic-rviz \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf \
    ros-noetic-urdf \
    ros-noetic-visualization-msgs \
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

COPY scripts/entrypoint.sh /ros_entrypoint_lab03.sh
RUN chmod +x /ros_entrypoint_lab03.sh

EXPOSE 6080

ENTRYPOINT ["/ros_entrypoint_lab03.sh"]
