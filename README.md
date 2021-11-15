# fusion
## daheng camera and delphi radar fusion

##### 融合

1.roslaunch camera_radar pre_fusion.launch

2.roslaunch delphi esr.launch

3.roslaunch camera_radar fusion.launch

##### 标定

1.roslaunch calibration calibration_pre.launch

2.roslaunch calibration camera_radar_calibration.launch

3.rosparam dump ~/fusion/src/calibration/result/params.yaml

录包
rosbag record -a -x "(.*)/compressed(.*)"

# 更新日志

##### 20210930

>实现基础融合功能和标定

##### 20211007

>新增显示雷达点属性功能

##### 20211008

>新增初步匹配功能，仅能运行

##### 20211011

>新增匈牙利算法匹配雷达点与yolo检测框

##### 20211012

>test包基本实现视觉检测帧间匹配

##### 20211014

>完善iou匹配，引入锚框做法

##### 20211025

>封装kvaser_interface

>新增单目测距

>改进融合包config.yaml，实现topic动态配置

>anchor_generate新增越界保护功能

##### 20211101

>函数接口优化

>匹配策略混合使用

##### 20211104

>测试版，无意义

##### 20211115

>自定义C++，使用缓存队列存放消息，多线程融合，完成ROS多话题融合

>新增lidar欧式聚类

