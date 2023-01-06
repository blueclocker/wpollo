cmake_minimum_required(VERSION 2.8.3)
project(adaptive_clustering)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
 roscpp 
 std_msgs 
 sensor_msgs 
 visualization_msgs 
 geometry_msgs 
 pcl_ros 
 rs_perception
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#add_message_files(FILES ClusterArray.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs )

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp std_msgs sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros rs_perception
)

add_executable(adaptive_clustering src/adaptive_clustering.cpp)
target_link_libraries(adaptive_clustering ${catkin_LIBRARIES}  ${OpenCV_LIBS})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(adaptive_clustering ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBS})
endif()
