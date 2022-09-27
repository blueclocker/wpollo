cmake_minimum_required(VERSION 3.0.2)
project(static_frenet)

add_compile_options(-std=c++11)

set(FRENET_SOURCES src/collision.cpp
                   src/epa.cpp
                   src/gjk.cpp
                   src/shape.cpp
                   src/vector2d.cpp)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
std_msgs
rs_perception
visualization_msgs
nav_msgs
osmmap
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs rs_perception visualization_msgs nav_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/frenet_optimal_trajectory.cpp ${FRENET_SOURCES})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

