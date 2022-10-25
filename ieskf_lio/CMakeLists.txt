cmake_minimum_required(VERSION 3.0.2)
project(ieskf_lio)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
SET(CMAKE_BUILD_TYPE Release)
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)


set(CUSTOM_TBB_DIR "${PROJECT_SOURCE_DIR}/thirdparty/tbb2018_20170726oss")

if (CUSTOM_TBB_DIR)
    set(TBB2018_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2018_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.7")
    include_directories(${TBB2018_INCLUDE_DIR})
    link_directories(${TBB2018_LIBRARY_DIR})
endif ()



find_package(catkin REQUIRED COMPONENTS
  cloud_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  dynamic_reconfigure 
)


generate_messages(
  DEPENDENCIES
  std_msgs
)





find_package(PCL REQUIRED QUIET)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED QUIET)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} custom_dynamic_reconfigure
   CATKIN_DEPENDS  dynamic_reconfigure tf
   DEPENDS eigen  PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  
)


add_executable(lidar_repub src/lidar_repub.cpp)
#add_dependencies(lidar_repub ${PROJECT_NAME}_gencfg)
target_link_libraries(lidar_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(lidar_repub cloud_msgs_gencpp)


add_executable(cari_lidar_repub src/cari_lidar_repub.cpp)
target_link_libraries(cari_lidar_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(cari_lidar_repub cloud_msgs_gencpp)

add_executable(cari_imu_repub src/cari_imu_repub.cpp)
target_link_libraries(cari_imu_repub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(cari_imu_repub cloud_msgs_gencpp)


#relocalization

add_executable(Preprocess src/relocalization/Preprocess_relocalization.cpp )
target_link_libraries(Preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(Preprocess cloud_msgs_gencpp)


add_executable(relocalization src/relocalization/globalRelocalization_ieskf.cpp src/Scancontext.cpp)
target_link_libraries(relocalization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} tbb glog)
add_dependencies(relocalization cloud_msgs_gencpp)



