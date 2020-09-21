cmake_minimum_required(VERSION 3.0.2)
project(float32multiarray_to_pointcloud2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
)

###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(float32multiarray_to_pointcloud2 src/float32multiarray_to_pointcloud2.cpp)
target_link_libraries(float32multiarray_to_pointcloud2 ${catkin_LIBRARIES})


