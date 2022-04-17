cmake_minimum_required(VERSION 2.8.3)
project(serial)
add_compile_options(-std=c++11)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
# Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  fsd_common_msgs
)

if(APPLE)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(FOUNDATION_LIBRARY Foundation)
endif()

if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        DEPENDS rt pthread
    )
else()
    # Otherwise normal call
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
 		CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 		DEPENDS system_lib
    )
endif()

## Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
if(APPLE)
	# If OSX
	list(APPEND serial_SRCS src/impl/unix.cc)
	list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(${PROJECT_NAME} ${serial_SRCS})
if(APPLE)
	target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
	target_link_libraries(${PROJECT_NAME} rt pthread)
else()
	target_link_libraries(${PROJECT_NAME} setupapi)
endif()

add_executable(serialPort src/serialPort.cpp)
add_dependencies(serialPort ${PROJECT_NAME} fsd_common_msgs_gencpp)
target_link_libraries(serialPort ${catkin_LIBRARIES} ${PROJECT_NAME})

## Include headers
include_directories(include ${catkin_INCLUDE_DIRS})

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)
