cmake_minimum_required(VERSION 2.8.3)
project(useeigen)


find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED COMPONENTS)
find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_lidar_calib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${Eigen_INCLUDE_DIRS}
)
include_directories( "/usr/include/eigen3" )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(useeigen1 useeigen.cpp)
target_link_libraries(useeigen1 ${catkin_LIBRARIES})
target_link_libraries(useeigen1 ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
#target_link_libraries (useeigen1 Eigen3::Eigen)
