cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_diff_car)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  rospy
  sensor_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/simple_diff_drive_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
