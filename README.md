# fusion
daheng camera and delphi radar fusion

融合
1.
roslaunch camera_radar pre_fusion.launch

2.
roslaunch delphi esr.launch

3.
roslaunch camera_radar fusion.launch

标定
1.
roslaunch calibration calibration_pre.launch

2.
roslaunch calibration camera_radar_calibration.launch

3.保存标定结果
rosparam dump ~/fusion/src/calibration/result/params.yaml

录包
rosbag record -a -x "(.*)/compressed(.*)"
