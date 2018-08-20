cmake_minimum_required(VERSION 2.8.3)
project(sensor_synchronization)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs message_filters)
find_package(PCL REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)
#find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
#  LIBRARIES multi_lidar_calib
#  LIBRARIES loam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 #${Eigen_INCLUDE_DIRS}
)
include_directories( "/usr/include/eigen3" )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(sensor_sync sensor_sync.cpp)
target_link_libraries(sensor_sync ${catkin_LIBRARIES})
target_link_libraries(sensor_sync ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
