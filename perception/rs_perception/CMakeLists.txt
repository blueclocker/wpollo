cmake_minimum_required( VERSION 2.8.3 )
project( rs_perception )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
		FILES
		PerceptionMsg.msg
		PerceptionListMsg.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(
		CATKIN_DEPENDS  std_msgs geometry_msgs message_runtime
)


