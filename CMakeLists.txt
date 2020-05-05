cmake_minimum_required(VERSION 2.8.3)
project(gazebo-data-annotator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  rospy
  sensor_msgs
  tf
)

# ==============================================
# Python scripts setup
# ==============================================
catkin_python_setup()
