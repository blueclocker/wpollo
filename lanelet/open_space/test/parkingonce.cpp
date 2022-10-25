/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-09 14:07:55
 * @LastEditTime: 2022-10-18 15:58:38
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parkingonce.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "parkingonce.h"

namespace apollo{
namespace planning{
ParkingOnce::ParkingOnce(ros::NodeHandle &nh)
{
    obs_pub_ = nh.advertise<visualization_msgs::MarkerArray>("obsmarkers", 1);
    path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path", 1);

    startpoint_sub_ = nh.subscribe("/initialpose", 1, &ParkingOnce::StartpointCallback, this);
    goalpoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &ParkingOnce::GoalpointCallback, this);
    
    has_obstacle_ = false;
    has_start_ = false;
    has_end_ = false;

    apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
    // std::unique_ptr<apollo::planning::HybridAStar> hybrid_test;
    hybrid_test_ = std::unique_ptr<apollo::planning::HybridAStar>(
        new apollo::planning::HybridAStar(planner_open_space_config_));

    XYbounds_.push_back(0.0);//xmin
    XYbounds_.push_back(80.0);//xmax
    XYbounds_.push_back(0.0);//ymin
    XYbounds_.push_back(50.0);//ymax

    setObs();

    while(nh.ok())
    {
        visualizationObstacles(obstacles_list_);
        ros::Rate(10).sleep();
        ros::spinOnce();
    }
}

ParkingOnce::~ParkingOnce()
{
}

void ParkingOnce::StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    sx_ = msg->pose.pose.position.x;
    sy_ = msg->pose.pose.position.y;
    sphi_ = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("get start point: x = %f, y = %f", sx_, sy_);
    has_start_ = true;

    if(has_end_ && has_start_) 
    {
        ROS_INFO("start plan ...");
        result_.clear();
        bool find = hybrid_test_->Plan(sx_, sy_, sphi_, ex_, ey_, ephi_, XYbounds_, obstacles_list_, &result_);
        if(!find) return;
        visualizationPath(result_);
        static tf::TransformBroadcaster transform_broadcaster;
        for (int i = 0; i < result_.x.size(); ++i) 
        {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(result_.x[i], result_.y[i], 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, result_.phi[i]);
            transform.setRotation(q);
            transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_link"));

            ros::Duration(0.05).sleep();
        }
    }
}

void ParkingOnce::GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ex_ = msg->pose.position.x;
    ey_ = msg->pose.position.y;
    ephi_ = tf::getYaw(msg->pose.orientation);
    ROS_INFO("get goal point: x = %f, y = %f", ex_, ey_);
    has_end_ = true;

    if(has_end_ && has_start_)
    {
        ROS_INFO("start plan ...");
        result_.clear();
        bool find = hybrid_test_->Plan(sx_, sy_, sphi_, ex_, ey_, ephi_, XYbounds_, obstacles_list_, &result_);
        if(!find) return;
        visualizationPath(result_);
        static tf::TransformBroadcaster transform_broadcaster;
        for (int i = 0; i < result_.x.size(); ++i) 
        {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(result_.x[i], result_.y[i], 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, result_.phi[i]);
            transform.setRotation(q);
            transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_link"));

            ros::Duration(0.05).sleep();
        }
    }
}

void ParkingOnce::setObs()
{
    ROS_INFO("set obstacles!");
    obstacles_list_.clear();
    std::vector<apollo::common::math::Vec2d> a_obstacle;
    a_obstacle.emplace_back(10.0, 13.0);
    a_obstacle.emplace_back(12.0, 13.0);
    a_obstacle.emplace_back(12.0, 18.0);
    a_obstacle.emplace_back(10.0, 18.0);
    a_obstacle.emplace_back(10.0, 13.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(16.0, 13.0);
    a_obstacle.emplace_back(18.0, 13.0);
    a_obstacle.emplace_back(18.0, 17.0);
    a_obstacle.emplace_back(16.0, 17.0);
    a_obstacle.emplace_back(16.0, 13.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(19.0, 12.0);
    a_obstacle.emplace_back(21.0, 12.0);
    a_obstacle.emplace_back(21.0, 17.0);
    a_obstacle.emplace_back(19.0, 17.0);
    a_obstacle.emplace_back(19.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(22.0, 11.0);
    a_obstacle.emplace_back(24.0, 11.0);
    a_obstacle.emplace_back(24.0, 15.0);
    a_obstacle.emplace_back(22.0, 15.0);
    a_obstacle.emplace_back(22.0, 11.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(28.0, 12.0);
    a_obstacle.emplace_back(30.0, 12.0);
    a_obstacle.emplace_back(30.0, 17.0);
    a_obstacle.emplace_back(28.0, 17.0);
    a_obstacle.emplace_back(28.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(33.0, 13.0);
    a_obstacle.emplace_back(35.0, 13.0);
    a_obstacle.emplace_back(35.0, 18.0);
    a_obstacle.emplace_back(33.0, 18.0);
    a_obstacle.emplace_back(33.0, 13.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(36.0, 12.0);
    a_obstacle.emplace_back(38.0, 12.0);
    a_obstacle.emplace_back(38.0, 16.0);
    a_obstacle.emplace_back(36.0, 16.0);
    a_obstacle.emplace_back(36.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(39.0, 13.0);
    a_obstacle.emplace_back(41.0, 13.0);
    a_obstacle.emplace_back(41.0, 17.0);
    a_obstacle.emplace_back(39.0, 17.0);
    a_obstacle.emplace_back(39.0, 13.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(42.0, 12.0);
    a_obstacle.emplace_back(44.0, 12.0);
    a_obstacle.emplace_back(44.0, 17.0);
    a_obstacle.emplace_back(42.0, 17.0);
    a_obstacle.emplace_back(42.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(48.0, 12.0);
    a_obstacle.emplace_back(50.0, 12.0);
    a_obstacle.emplace_back(50.0, 16.0);
    a_obstacle.emplace_back(48.0, 16.0);
    a_obstacle.emplace_back(48.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(51.0, 12.0);
    a_obstacle.emplace_back(53.0, 12.0);
    a_obstacle.emplace_back(53.0, 17.0);
    a_obstacle.emplace_back(51.0, 17.0);
    a_obstacle.emplace_back(51.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(58.0, 13.0);
    a_obstacle.emplace_back(60.0, 13.0);
    a_obstacle.emplace_back(60.0, 18.0);
    a_obstacle.emplace_back(58.0, 18.0);
    a_obstacle.emplace_back(58.0, 13.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(61.0, 12.0);
    a_obstacle.emplace_back(63.0, 12.0);
    a_obstacle.emplace_back(63.0, 16.0);
    a_obstacle.emplace_back(61.0, 16.0);
    a_obstacle.emplace_back(61.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(64.0, 13.0);
    a_obstacle.emplace_back(66.0, 13.0);
    a_obstacle.emplace_back(66.0, 17.0);
    a_obstacle.emplace_back(64.0, 17.0);
    a_obstacle.emplace_back(64.0, 13.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(70.0, 11.0);
    a_obstacle.emplace_back(72.0, 11.0);
    a_obstacle.emplace_back(72.0, 15.0);
    a_obstacle.emplace_back(70.0, 15.0);
    a_obstacle.emplace_back(70.0, 11.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(73.0, 12.0);
    a_obstacle.emplace_back(75.0, 12.0);
    a_obstacle.emplace_back(75.0, 16.0);
    a_obstacle.emplace_back(73.0, 16.0);
    a_obstacle.emplace_back(73.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(76.0, 12.0);
    a_obstacle.emplace_back(78.0, 12.0);
    a_obstacle.emplace_back(78.0, 17.0);
    a_obstacle.emplace_back(76.0, 17.0);
    a_obstacle.emplace_back(76.0, 12.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(3.0, 23.0);
    a_obstacle.emplace_back(5.0, 23.0);
    a_obstacle.emplace_back(5.0, 28.0);
    a_obstacle.emplace_back(3.0, 28.0);
    a_obstacle.emplace_back(3.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(6.0, 22.0);
    a_obstacle.emplace_back(8.0, 22.0);
    a_obstacle.emplace_back(8.0, 26.0);
    a_obstacle.emplace_back(6.0, 26.0);
    a_obstacle.emplace_back(6.0, 22.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(9.0, 23.0);
    a_obstacle.emplace_back(11.0, 23.0);
    a_obstacle.emplace_back(11.0, 27.0);
    a_obstacle.emplace_back(9.0, 27.0);
    a_obstacle.emplace_back(9.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(15.0, 23.0);
    a_obstacle.emplace_back(17.0, 23.0);
    a_obstacle.emplace_back(17.0, 27.0);
    a_obstacle.emplace_back(15.0, 27.0);
    a_obstacle.emplace_back(15.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(18.0, 24.0);
    a_obstacle.emplace_back(20.0, 24.0);
    a_obstacle.emplace_back(20.0, 28.0);
    a_obstacle.emplace_back(18.0, 28.0);
    a_obstacle.emplace_back(18.0, 24.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(21.0, 24.0);
    a_obstacle.emplace_back(23.0, 24.0);
    a_obstacle.emplace_back(23.0, 29.0);
    a_obstacle.emplace_back(21.0, 29.0);
    a_obstacle.emplace_back(21.0, 24.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(24.0, 23.0);
    a_obstacle.emplace_back(26.0, 23.0);
    a_obstacle.emplace_back(26.0, 27.0);
    a_obstacle.emplace_back(24.0, 27.0);
    a_obstacle.emplace_back(24.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(30.0, 24.0);
    a_obstacle.emplace_back(32.0, 24.0);
    a_obstacle.emplace_back(32.0, 29.0);
    a_obstacle.emplace_back(30.0, 29.0);
    a_obstacle.emplace_back(30.0, 24.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(33.0, 23.0);
    a_obstacle.emplace_back(35.0, 23.0);
    a_obstacle.emplace_back(35.0, 27.0);
    a_obstacle.emplace_back(33.0, 27.0);
    a_obstacle.emplace_back(33.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(36.0, 24.0);
    a_obstacle.emplace_back(38.0, 24.0);
    a_obstacle.emplace_back(38.0, 28.0);
    a_obstacle.emplace_back(36.0, 28.0);
    a_obstacle.emplace_back(36.0, 24.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(47.0, 23.0);
    a_obstacle.emplace_back(49.0, 23.0);
    a_obstacle.emplace_back(49.0, 28.0);
    a_obstacle.emplace_back(47.0, 28.0);
    a_obstacle.emplace_back(47.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(50.0, 22.0);
    a_obstacle.emplace_back(52.0, 22.0);
    a_obstacle.emplace_back(52.0, 26.0);
    a_obstacle.emplace_back(50.0, 26.0);
    a_obstacle.emplace_back(50.0, 22.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(53.0, 23.0);
    a_obstacle.emplace_back(55.0, 23.0);
    a_obstacle.emplace_back(55.0, 27.0);
    a_obstacle.emplace_back(53.0, 27.0);
    a_obstacle.emplace_back(53.0, 23.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(56.0, 22.0);
    a_obstacle.emplace_back(58.0, 22.0);
    a_obstacle.emplace_back(58.0, 27.0);
    a_obstacle.emplace_back(56.0, 27.0);
    a_obstacle.emplace_back(56.0, 22.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(59.0, 21.0);
    a_obstacle.emplace_back(61.0, 21.0);
    a_obstacle.emplace_back(61.0, 25.0);
    a_obstacle.emplace_back(59.0, 25.0);
    a_obstacle.emplace_back(59.0, 21.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(62.0, 22.0);
    a_obstacle.emplace_back(64.0, 22.0);
    a_obstacle.emplace_back(64.0, 26.0);
    a_obstacle.emplace_back(62.0, 26.0);
    a_obstacle.emplace_back(62.0, 22.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(65.0, 22.0);
    a_obstacle.emplace_back(67.0, 22.0);
    a_obstacle.emplace_back(67.0, 27.0);
    a_obstacle.emplace_back(65.0, 27.0);
    a_obstacle.emplace_back(65.0, 22.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(11.0, 33.0);
    a_obstacle.emplace_back(13.0, 33.0);
    a_obstacle.emplace_back(13.0, 38.0);
    a_obstacle.emplace_back(11.0, 38.0);
    a_obstacle.emplace_back(11.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(14.0, 32.0);
    a_obstacle.emplace_back(16.0, 32.0);
    a_obstacle.emplace_back(16.0, 36.0);
    a_obstacle.emplace_back(14.0, 36.0);
    a_obstacle.emplace_back(14.0, 32.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(20.0, 33.0);
    a_obstacle.emplace_back(22.0, 33.0);
    a_obstacle.emplace_back(22.0, 38.0);
    a_obstacle.emplace_back(20.0, 38.0);
    a_obstacle.emplace_back(20.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(26.0, 33.0);
    a_obstacle.emplace_back(28.0, 33.0);
    a_obstacle.emplace_back(28.0, 38.0);
    a_obstacle.emplace_back(26.0, 38.0);
    a_obstacle.emplace_back(26.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(29.0, 32.0);
    a_obstacle.emplace_back(31.0, 32.0);
    a_obstacle.emplace_back(31.0, 36.0);
    a_obstacle.emplace_back(29.0, 36.0);
    a_obstacle.emplace_back(29.0, 32.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(32.0, 33.0);
    a_obstacle.emplace_back(34.0, 33.0);
    a_obstacle.emplace_back(34.0, 37.0);
    a_obstacle.emplace_back(32.0, 37.0);
    a_obstacle.emplace_back(32.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(35.0, 33.0);
    a_obstacle.emplace_back(37.0, 33.0);
    a_obstacle.emplace_back(37.0, 38.0);
    a_obstacle.emplace_back(35.0, 38.0);
    a_obstacle.emplace_back(35.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(41.0, 33.0);
    a_obstacle.emplace_back(43.0, 33.0);
    a_obstacle.emplace_back(43.0, 38.0);
    a_obstacle.emplace_back(41.0, 38.0);
    a_obstacle.emplace_back(41.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(47.0, 33.0);
    a_obstacle.emplace_back(49.0, 33.0);
    a_obstacle.emplace_back(49.0, 37.0);
    a_obstacle.emplace_back(47.0, 37.0);
    a_obstacle.emplace_back(47.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(50.0, 33.0);
    a_obstacle.emplace_back(52.0, 33.0);
    a_obstacle.emplace_back(52.0, 38.0);
    a_obstacle.emplace_back(50.0, 38.0);
    a_obstacle.emplace_back(50.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(56.0, 32.0);
    a_obstacle.emplace_back(58.0, 32.0);
    a_obstacle.emplace_back(58.0, 36.0);
    a_obstacle.emplace_back(56.0, 36.0);
    a_obstacle.emplace_back(56.0, 32.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(59.0, 33.0);
    a_obstacle.emplace_back(61.0, 33.0);
    a_obstacle.emplace_back(61.0, 37.0);
    a_obstacle.emplace_back(59.0, 37.0);
    a_obstacle.emplace_back(59.0, 33.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(23.0, 43.0);
    a_obstacle.emplace_back(25.0, 43.0);
    a_obstacle.emplace_back(25.0, 48.0);
    a_obstacle.emplace_back(23.0, 48.0);
    a_obstacle.emplace_back(23.0, 43.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(26.0, 42.0);
    a_obstacle.emplace_back(28.0, 42.0);
    a_obstacle.emplace_back(28.0, 46.0);
    a_obstacle.emplace_back(26.0, 46.0);
    a_obstacle.emplace_back(26.0, 42.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(29.0, 43.0);
    a_obstacle.emplace_back(31.0, 43.0);
    a_obstacle.emplace_back(31.0, 47.0);
    a_obstacle.emplace_back(29.0, 47.0);
    a_obstacle.emplace_back(29.0, 43.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(32.0, 42.0);
    a_obstacle.emplace_back(34.0, 42.0);
    a_obstacle.emplace_back(34.0, 47.0);
    a_obstacle.emplace_back(32.0, 47.0);
    a_obstacle.emplace_back(32.0, 42.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(35.0, 41.0);
    a_obstacle.emplace_back(37.0, 41.0);
    a_obstacle.emplace_back(37.0, 45.0);
    a_obstacle.emplace_back(35.0, 45.0);
    a_obstacle.emplace_back(35.0, 41.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(38.0, 42.0);
    a_obstacle.emplace_back(40.0, 42.0);
    a_obstacle.emplace_back(40.0, 46.0);
    a_obstacle.emplace_back(38.0, 46.0);
    a_obstacle.emplace_back(38.0, 42.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(41.0, 42.0);
    a_obstacle.emplace_back(43.0, 42.0);
    a_obstacle.emplace_back(43.0, 47.0);
    a_obstacle.emplace_back(41.0, 47.0);
    a_obstacle.emplace_back(41.0, 42.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(58.0, 42.0);
    a_obstacle.emplace_back(60.0, 42.0);
    a_obstacle.emplace_back(60.0, 47.0);
    a_obstacle.emplace_back(58.0, 47.0);
    a_obstacle.emplace_back(58.0, 42.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(61.0, 41.0);
    a_obstacle.emplace_back(63.0, 41.0);
    a_obstacle.emplace_back(63.0, 45.0);
    a_obstacle.emplace_back(61.0, 45.0);
    a_obstacle.emplace_back(61.0, 41.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(67.0, 41.0);
    a_obstacle.emplace_back(69.0, 41.0);
    a_obstacle.emplace_back(69.0, 46.0);
    a_obstacle.emplace_back(67.0, 46.0);
    a_obstacle.emplace_back(67.0, 41.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(70.0, 40.0);
    a_obstacle.emplace_back(72.0, 40.0);
    a_obstacle.emplace_back(72.0, 44.0);
    a_obstacle.emplace_back(70.0, 44.0);
    a_obstacle.emplace_back(70.0, 40.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(73.0, 41.0);
    a_obstacle.emplace_back(75.0, 41.0);
    a_obstacle.emplace_back(75.0, 45.0);
    a_obstacle.emplace_back(73.0, 45.0);
    a_obstacle.emplace_back(73.0, 41.0);
    obstacles_list_.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(76.0, 41.0);
    a_obstacle.emplace_back(78.0, 41.0);
    a_obstacle.emplace_back(78.0, 46.0);
    a_obstacle.emplace_back(76.0, 46.0);
    a_obstacle.emplace_back(76.0, 41.0);
    obstacles_list_.emplace_back(a_obstacle);

    has_obstacle_ = true;
    // visualizationObstacles(obstacles_list_);
}

void ParkingOnce::visualizationPath(apollo::planning::HybridAStartResult &path)
{
    if(path.x.size() == 0) return;
    visualization_msgs::MarkerArray paths;
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    // m.lifetime = ros::Duration(0.1);
    m.ns = "path";
    m.id = 1;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    m.color.b = 1.0;
    m.scale.x = 0.2;
    CHECK_EQ(path.x.size(), path.y.size());
    // CHECK_EQ(path.x.size(), path.phi.size());

    visualization_msgs::Marker vehicle;
    vehicle.header.frame_id = "map";
    vehicle.header.stamp = ros::Time::now();
    vehicle.type = visualization_msgs::Marker::CUBE;
    // vehicle.id = static_cast<int>(i / vehicle_interval);
    vehicle.ns = "vehicle";
    vehicle.scale.x = 4.933;//2.11
    vehicle.scale.y = 2.11;//4.933
    vehicle.scale.z = 0.01;
    vehicle.color.a = 0.1;

    vehicle.color.r = 1.0;
    vehicle.color.b = 1.0;
    vehicle.color.g = 0.0;

    for(int i = 0; i < path.x.size(); ++i)
    {
        geometry_msgs::Point a;
        a.x = path.x[i];
        a.y = path.y[i];
        a.z = 0;
        m.points.push_back(a);

        vehicle.id = i;
        vehicle.pose.position.x = a.x;
        vehicle.pose.position.y = a.y;
        vehicle.pose.position.z = 0.0;
        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path.phi[i]);
        paths.markers.push_back(vehicle);
    }

    paths.markers.push_back(m);
    path_pub_.publish(paths);
}

void ParkingOnce::visualizationObstacles(const std::vector<std::vector<apollo::common::math::Vec2d>> &obstacles_lists)
{
    visualization_msgs::MarkerArray obses;
    visualization_msgs::Marker obs;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.action = visualization_msgs::Marker::ADD;
    obs.type = visualization_msgs::Marker::LINE_STRIP;
    obs.ns = "obstacle";
    // obs.lifetime = ros::Duration(0.1);
    obs.pose.orientation.w = 1.0;
    obs.color.a = 1.0;
    obs.color.r = 1.0;
    obs.scale.x = 0.1;
    int index = 0;
    // obs.id = 1;
    for(auto p : obstacles_lists)
    {
        obs.points.clear();
        obs.id = index++;
        for(auto pp : p)
        {
            geometry_msgs::Point a;
            a.x = pp.x();
            a.y = pp.y();
            a.z = 0;
            obs.points.push_back(a);
        }
        obses.markers.push_back(obs);
    }
    obs_pub_.publish(obses);
}

void ParkingOnce::run()
{
    // visualizationObstacles(obstacles_list_);
    ros::spin();
}


}
}