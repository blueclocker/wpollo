# WPOLLO

#### 相机毫米波雷达融合

##### 毫米波雷达与视觉融合 camera_radar
* 1.打开相机、毫米波雷达（包括kvaser）和YOLO
```
roslaunch camera_radar pre_fusion.launch
```
* 2.启动融合程序
```
roslaunch camera_radar fusion.launch
```

##### 毫米波雷达与视觉外参标定 calibration
* 1.启动相机和毫米波雷达
```
roslaunch calibration calibration_pre.launch
```
* 2.打开ROS动态调参界面
```
roslaunch calibration camera_radar_calibration.launch
```
* 3.保存参数
```
rosparam dump ~/fusion/src/calibration/result/params.yaml
```
* 录包
```
rosbag record -a -x "(.*)/compressed(.*)"
```

##### YOLO darknet_ros
* [darkent_ros](https://github.com/leggedrobotics/darknet_ros)

##### 大恒相机驱动 galaxy_camera
```
roslaunch galaxy_camera MER-231.launch
```

##### 德尔福毫米波雷达驱动 delphi_driver
```
roslaunch kvaser_interface kvaser_can_bridge.launch
```

#### 激光雷达

##### 激光雷达按距离分割点云再使用pcl库欧式聚类 lidar
```
roslaunch lidar test.launch
```

##### autoware提取的激光雷达slam及其组件 liadr_localizer ndt prius_description
* [autoware](https://github.com/Autoware-AI/autoware.ai)
* 小车模型prius_description
* slam
```
roslaunch lidar_localizer ndt_mapping.launch
```

#### 全局路径规划 plan
* 在栅格地图实现A*算法
```
roslaunch plan plan_test.launch
```
* 根据xml文件绘制节点拓扑图发布成ROS的markerarray格式
```
roslaunch plan drawmap.launch
```

#### 工具箱

##### pcd转pgm pcd2pgm
* 读取pcd将其转化为nav_msgs::Occupancy栅格地图，再使用map_server保存
```
roscore
rosrun pcd2pgm pcd2topic
rosrun map_server map_saver -f bitmap
```

##### 多功能ROS工具箱 srv_tools

##### 绘制节点拓扑图 graph_tool
* 根据pcd点云图或栅格地图，手工绘制节点拓扑图
- 启动标定程序，通过rviz 2D Pose Estimate 选定关键点，并自动给出序号
```
roslaunch graph_tool graph_tool.launch
```
* 使用说明：
- 默认开启捕捉模式，捕捉半径在launch文件设置，**searchradius**
- 添加两点连线，勾选add_path
> 在rviz使用**initialpos**，先选定第一个点，在选定第二个点，自动实现连线
- 删除一个点，勾选delete_point
> 在rviz使用**initialpos**，选定要删去的点
- 删除一条线，勾选delete_path
> 与添加连线方法相同

* 注：
- 连线与删除过程，对两点的顺序不敏感
- rviz可能会出现未识别到用户鼠标点击的情况，建议查看终端输出信息，再酌情处理
- 不按照使用说明使用本工具，可能造成无法预知的问题

#### Lanelet

##### adam_shan 改进版 ad_with_lanelet2

##### 高精地图ROS全局规划模块 lanelet
* 启动rviz可视化高精地图以及全局路径规划
```
roslaunch osmmap osmmap.launch
```

#####  串口读取模块  serial
* 打开USB串口权限，如果需要，目前GPS不能从USB读取数据，原因不明
```
sudo chmod 777 /dev/ttyUSB0
rosrun serial serialPort
```

#####  串口数据转标准GPS数据格式模块 odometry_publisher
* 全部GPS/IMU信息，发布fsd_common_msgs::comb, topic = "/comb"
* 里程计信息，发布fsd_common_msgs::Gnss, topic = "/gnss_odom"
* IMU，发布sensor_msgs::Imu, topic = "/gnss_imu"
* ROS标准GPS格式，发布sensor_msgs::NavSatFix, topic = "/gps/fix"
```
rosrun odometry_publishergnss_odom_pub
```

#####  各种自定义msg fsd_common_msgs
* 各种msg，在CmakeList.txt和package.xml设置依赖可以实现跨包调用
- CmakeList.txt
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  fsd_common_msgs
)

add_dependencies(serialPort ${PROJECT_NAME} fsd_common_msgs_gencpp)
```

- package.xml
```
<build_depend>fsd_common_msgs</build_depend>
<exec_depend>fsd_common_msgs</exec_depend>
```

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

##### 20220110
* 优化graph_tool使用体验，实现建图过程只使用鼠标
* 优化graph_tool对点的编号问题，当删去的是最后一点时回收该id，暂时未能解决id跳跃的问题
* 优化graph_tool保存问题，每次存储新的文件，存储格式为**文件名-当前时间.xml**

##### 20220307
* 新增lanelet/osmmap模块，解析lanelet2标准osm地图得到ros下的可视化效果
- 目前完成道路边界显示，道路中心线计算与显示

##### 20220322
* 优化lanelet/osmmap模块
- 完成基础A*算法迁移，现阶段基于生成的道路中心线做全局规划，道路信号标志暂未实现与道路中心线关联
- 支持通过rviz，通过**initialpose**选取道路起始点，通过**move_base_simple/goal**选取道路终点
- 全局路径规划目前只精确到路段，后续优化到具体某一道路中心点
- 预留GPS接口，后续可以与定位模块联动，接口暂未确定

##### 20220409
* 优化lanelet/osmmap模块
- 全局路径规划可以精确到具体某道路中心线的一点
- 交通信号标志已经完成与道路中心线的关联，实现关联的程序分布在map_relation.cpp和centerway.cpp
- 道路中心线的id = 相应的道路边界的id = xml原始relation的id
- 实现某点到该道路两侧的距离估计
- 使用三次样条插值实现路径平滑，**缺少车辆动力学约束**
- 发布导航信息，当前只发布起点所在路段的道路信息，具体信息内容参见navigation.msg
- 当前重规划存在浪费算力问题，后期考虑优化成基于增量模式，只重规划少部分路径

##### 20220417
* 新增serial、odometry_publisher和fsd_common_msgs
- 用于实时从串口读取GPS数据
* 优化lanelet/osmmap模块
- 部分解决重规划问题，如果原始的规划数据满足要求则不再重复A*，但不能更新cost权重
- 修正原node模块墨卡托投影问题，现使用GeographicLib库，已集成到osmmap模块，坐标转换接口在mapio内
- 优化原始点经度、维度、高度数据输入方式，现在launch文件中设置
- 本模块初步具有实时全局规划并提供导航信息的能力，但还存在一些细节问题，比如：当车走到最后一段路时，全局路径失效
- 新增地图若干，有多车道、双向等场景
- 在lanelet定位具体点坐标的方式存在缺陷，在环形道路会出现问题

##### 20220420
* 优化lanelet/osmmap模块
- 优化全局路径搜索，如果当前点与目标点在同一路段，可以获得该路段的具体中心点的导航
- 支持同向多车道，发布的导航信息会包括当前路段的可行相邻路段
- 初步增加换道功能，目前只能在某些固定点换道
- 新增停车区接口，但还没连接到地图上
- lanelet定位具体点的方式不变，只需绘制地图时弯道转角不超过180度，则当前程序可行
- 优化部分接口

##### 20220423
* 优化lanelet/osmmap模块
- 新增初步换道策略，偶尔出现问题，原因暂时不明确
- 全面优化各函数接口，加入const限定
- map_io和map_core分离，在map_core中调用map_io，预置地图基础四类元素的常量指针
- 优化地图输出信息，目前包括导航信息、车辆状态信息，还需完善车道信息
- 下一版本预计解决停车位问题、地图数据传输问题

##### 20220912
* 新增jps2d和jps3d两种搜索方式
* 更改path_optimer_2为path_boost，改输入地图格式为pcl点云
* 下一版本大改osmmap，预计支持多模态路径规划

##### 20220927
* 新增open_space泊车功能，使用apollo混合A*
* osmmap导航方式优化，当超过地图范围和无法搜索出可行道路时，取和车头朝向相同的最近lanelet作为定位结果，尝试再次A*
