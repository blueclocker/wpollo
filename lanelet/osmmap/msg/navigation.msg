uint8 STRAIGHT=1
uint8 LEFT=2
uint8 RIGHT=3

std_msgs/Header header
uint32[] CurrentSequenceIDs  #当前可行驶车道
uint32   TargetSequeceIDs    #当前推荐车道
float32 SpeedLimits          #车道级限速信息 m/s
float32 ToLeftDistance	     #到左侧距离 m
float32 ToRightDistance	     #到右侧距离 m
uint8   direction	     #道路方向
float32 IntersectionDistance #下一次转弯距离 m
regulatoryelement[] TrafficSign #交通信号标志

