/*
 * @Author: wpbit
 * @Date: 2021-09-25 20:20:10
 * @LastEditTime: 2021-09-29 21:27:14
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/calibration/src/server.cpp
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include "calibration/TutorialsConfig.h"

ros::Publisher pub;
void callback(dynamic_calibration::TutorialsConfig &config)
{
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f",
           config.x,
           config.y,
           config.z,
           config.theta_x,
           config.theta_y,
	   config.theta_z);
  geometry_msgs::Twist transform_;
  transform_.linear.x = config.x;
  transform_.linear.y = config.y;
  transform_.linear.z = config.z;
  transform_.angular.x = config.theta_x;
  transform_.angular.y = config.theta_y;
  transform_.angular.z = config.theta_z;
  pub.publish(transform_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_calibration");
  ros::NodeHandle nh("~");
  pub = nh.advertise<geometry_msgs::Twist>("radar_transform_camera", 100);
  dynamic_reconfigure::Server<dynamic_calibration::TutorialsConfig> server;
  dynamic_reconfigure::Server<dynamic_calibration::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1); //绑定回调函数
  server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
  ROS_INFO("Spinning node");
  ros::spin(); //服务器循环监听重配置请求，当服务器收到重配置请求的时候，就会自动调用回调函数
  return 0;
}
