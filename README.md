# fusion

#### 相机毫米波雷达融合

##### 毫米波雷达与视觉融合 camera_radar
* 1.打开相机、毫米波雷达（包括kvaser）和YOLO
'''
roslaunch camera_radar pre_fusion.launch
'''
* 2.启动融合程序
'''
roslaunch camera_radar fusion.launch
'''

##### 毫米波雷达与视觉外参标定 calibration
* 1.启动相机和毫米波雷达
'''
roslaunch calibration calibration_pre.launch
'''
* 2.打开ROS动态调参界面
'''
roslaunch calibration camera_radar_calibration.launch
'''
* 3.保存参数
'''
rosparam dump ~/fusion/src/calibration/result/params.yaml
'''
* 录包
'''
rosbag record -a -x "(.*)/compressed(.*)"
'''

##### YOLO darknet_ros
* [darkent_ros](https://github.com/leggedrobotics/darknet_ros)

##### 大恒相机驱动 galaxy_camera
'''
roslaunch galaxy_camera MER-231.launch
'''

##### 德尔福毫米波雷达驱动 delphi_driver
'''
roslaunch kvaser_interface kvaser_can_bridge.launch
'''

#### 激光雷达

##### 激光雷达按距离分割点云再使用pcl库欧式聚类 lidar
'''
roslaunch lidar test.launch
'''

##### autoware提取的激光雷达slam及其组件 liadr_localizer ndt prius_description
* [autoware](https://github.com/Autoware-AI/autoware.ai)
* 小车模型prius_description
* slam
'''
roslaunch lidar_localizer ndt_mapping.launch
'''

#### 全局路径规划 plan
* 在栅格地图实现A*算法
'''
roslaunch plan plan_test.launch
'''
* 根据xml文件绘制节点拓扑图发布成ROS的markerarray格式
'''
roslaunch plan drawmap.launch
'''

#### 工具箱

##### pcd转pgm pcd2pgm
* 读取pcd将其转化为nav_msgs::Occupancy栅格地图，再使用map_server保存
'''
roscore
rosrun pcd2pgm pcd2topic
rosrun map_server map_saver -f bitmap
'''

##### 多功能ROS工具箱 srv_tools

##### 绘制节点拓扑图 graph_tool
* 根据pcd点云图或栅格地图，手工绘制节点拓扑图
- 启动标定程序，通过rviz 2D Pose Estimate 选定关键点，并自动给出序号
'''
roslaunch graph_tool graph_tool.launch
'''
- 建立X，Y两点连线，并保存两点关系
'''
rosrun graph_tool line_client X Y
'''
- 删除X点(及其关联的线)
'''
rosrun graph_tool deletepoint_client X
'''
- 删除X Y线
'''
rosrun graph_tool deletepath_client X Y
'''

#### 开发版 test
**!!!!!bug多!!!!!**
- 匈牙利算法
- 多线程控制ROS多回调函数
- 等


# 更新日志

##### 20210930
* 实现基础融合功能和标定

##### 20211007
* 新增显示雷达点属性功能

##### 20211008
* 新增初步匹配功能，仅能运行

##### 20211011
* 新增匈牙利算法匹配雷达点与yolo检测框

##### 20211012
* test包基本实现视觉检测帧间匹配

##### 20211014
* 完善iou匹配，引入锚框做法

##### 20211025
* 封装kvaser_interface
* 新增单目测距
* 改进融合包config.yaml，实现topic动态配置
* anchor_generate新增越界保护功能

##### 20211101
* 函数接口优化
* 匹配策略混合使用

##### 20211104
* 测试版，无意义

##### 20211115
* 自定义C++，使用缓存队列存放消息，多线程融合，完成ROS多话题融合
* 新增lidar欧式聚类

##### 20211130
* 新增plan模块，实现A*全局规划，globalplan节点实现简单仿真效果

##### 20211229
* 新增lidar_localizer模块，简化cpu版autoware激光slam
* 新增autoware小车模型prius_description
* 新增pac2pgm模块，读取pcd点云发布nav_msgs::OccupancyGrid格式，还需map_saver保存pgm和yaml文件
* 新增graph_tool模块，交互性标注slam地图得到节点拓扑图并保存成xml格式


##### 20220104
* 更新graph_tool模块，实现双模功能。建立全新xml地图**isnew**=true，使用已有xml地图再编辑**isnew**=false
* 更新graph_tool模块，新增删除点、删除线的方法。

##### 20220106
* 新增srv_tools模块
* graph_tool模块加入原始点云地图
