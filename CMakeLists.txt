cmake_minimum_required(VERSION 2.8.3)
project(gazebo_data_annotator)

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


# ==============================================
# Install
# ==============================================
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
