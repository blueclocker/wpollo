# This message describes the complete vehicle state
# orientation = orientation of the vehicle using euler angles

Header header
string child_frame_id

geometry_msgs/Point position
geometry_msgs/Vector3 orientation
geometry_msgs/Twist velocity
geometry_msgs/Twist acceleration
