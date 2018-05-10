cmake_minimum_required(VERSION 2.8.3)
project(multi_lidar_calib)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(PCL REQUIRED COMPONENTS)
#find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_lidar_calib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 #${Eigen_INCLUDE_DIRS}
)
include_directories( "/usr/include/eigen3" )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(read_data multi_lidar_calib.cpp)
target_link_libraries(read_data ${catkin_LIBRARIES})
target_link_libraries(read_data ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
