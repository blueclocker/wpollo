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
