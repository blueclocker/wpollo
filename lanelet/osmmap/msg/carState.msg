std_msgs/Header header
bool inMap			#是否在地图范围内
bool isEndExist			#是否有有效终点
bool isFindPath			#是否找到路径
uint32 laneletid		#当前所在车道ID
geometry_msgs/Pose carPose	#车辆位姿
geometry_msgs/Vector3 velocity	#速度
geometry_msgs/Vector3 Accell	#加速度
