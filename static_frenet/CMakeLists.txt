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
message_generation
dynamic_reconfigure
)

find_package(PCL 1.7 REQUIRED)

add_message_files(
FILES 
Track.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs rs_perception visualization_msgs nav_msgs message_runtime dynamic_reconfigure
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${EIGEN3_INCLUDE_DIR})
include_directories(include ${OpenCV_INCLUDE_DIRS})

# add_library(geographicLib
#     GeographicLib/Geocentric.cpp
#     GeographicLib/LocalCartesian.cpp
#     GeographicLib/Math.cpp
# )

add_executable(${PROJECT_NAME}_node src/frenet_optimal_trajectory.cpp ${FRENET_SOURCES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # geographicLib
)

