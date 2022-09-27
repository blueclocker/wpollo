/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-20 21:38:33
 * @LastEditTime: 2022-09-25 16:51:16
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parkingflow.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "parkingflow.h"

namespace apollo{
namespace planning{
ParkingFlow::ParkingFlow(ros::NodeHandle &nh)
{
    obs_pub_ = nh.advertise<visualization_msgs::MarkerArray>("obsmarkers", 1);
    path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path", 1);

    obs_sub_ = nh.subscribe("/adaptive_clustering/markers_tubao", 1, &ParkingFlow::obsCallback, this);
    carstate_sub_ = nh.subscribe("/mapio/carstate_info", 1, &ParkingFlow::carstateCallback, this);

    has_obstacle_ = false;
    has_start_ = false;
    has_end_ = false;
}

ParkingFlow::~ParkingFlow()
{
}

void ParkingFlow::init()
{
    apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
    // std::unique_ptr<apollo::planning::HybridAStar> hybrid_test;
    hybrid_test_ = std::unique_ptr<apollo::planning::HybridAStar>(
        new apollo::planning::HybridAStar(planner_open_space_config_));

    XYbounds_.push_back(-50.0);//xmin
    XYbounds_.push_back(50.0);//xmax
    XYbounds_.push_back(-50.0);//ymin
    XYbounds_.push_back(50.0);//ymax
}

void ParkingFlow::run()
{
    ros::spin();
}

void ParkingFlow::obsCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    if(car_pose_.position.x == 0 && car_pose_.position.y == 0) return;
    ROS_INFO("get obstacles!");
    obstacles_list_.clear();
    for(int i = 0; i < msg->markers.size(); ++i)
    {
        std::vector<apollo::common::math::Vec2d> a_obstacle;
        for(int j = 0; j < msg->markers[i].points.size(); ++j)
        {
            apollo::common::math::Vec2d point;
            double xv = msg->markers[i].points[j].x;
            double yv = msg->markers[i].points[j].y;
            double phiv = tf::getYaw(car_pose_.orientation);
            double xo = xv*cos(phiv) - yv*sin(phiv) + car_pose_.position.x;
            double yo = xv*sin(phiv) + yv*cos(phiv) + car_pose_.position.y;
            point.set_x(xo);
            point.set_y(yo);
            a_obstacle.emplace_back(point);
        }
        // obstacles_list_.emplace_back(a_obstacle);
    }
    has_obstacle_ = true;
    visualizationObstacles(obstacles_list_);
}

void ParkingFlow::carstateCallback(const osmmap::CarState::ConstPtr &msg)
{
    double sx = msg->carPose.position.x;
    double sy = msg->carPose.position.y;
    double sphi = tf::getYaw(msg->carPose.orientation);
    car_pose_ = msg->carPose;
    double ex = msg->nextpoint.x;
    double ey = msg->nextpoint.y;
    double ephi = msg->nextpoint.z;
    has_start_ = true;
    if(msg->endPose.position.x != 0 && msg->endPose.position.y != 0) has_end_ = true;

    //XYbounds_
    XYbounds_[0] = sx - 10;
    XYbounds_[1] = sx + 10;
    XYbounds_[2] = sy - 5;
    XYbounds_[3] = sy + 5;
    if(msg->endPose.position.x > XYbounds_[0] && msg->endPose.position.x < XYbounds_[1] && 
       msg->endPose.position.y > XYbounds_[2] && msg->endPose.position.y < XYbounds_[3])
    {
        ROS_INFO("near goal point!");
        ex = msg->endPose.position.x;
        ey = msg->endPose.position.y;
        ephi = tf::getYaw(msg->endPose.orientation);
    }
    if(ex == 0 && ey == 0 && ephi == 0) return;

    //plan
    if(has_start_ && has_end_)
    {
        ROS_INFO("start plan ...");
        result_.clear();
        hybrid_test_->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_list_, &result_);
        visualizationPath(result_);
        // visualizationObstacles(obstacles_list_);
    }
    // visualizationObstacles(obstacles_list_);
}

void ParkingFlow::visualizationObstacles(const std::vector<std::vector<apollo::common::math::Vec2d>> &obstacles_lists)
{
    visualization_msgs::MarkerArray obses;
    visualization_msgs::Marker obs;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.action = visualization_msgs::Marker::ADD;
    obs.type = visualization_msgs::Marker::LINE_STRIP;
    obs.ns = "obstacle";
    obs.lifetime = ros::Duration(0.1);
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

void ParkingFlow::visualizationPath(apollo::planning::HybridAStartResult &path)
{
    if(path.x.size() == 0) return;
    visualization_msgs::MarkerArray paths;
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.lifetime = ros::Duration(0.1);
    m.ns = "path";
    m.id = 1;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    m.color.b = 1.0;
    m.scale.x = 0.25;
    CHECK_EQ(path.x.size(), path.y.size());
    // CHECK_EQ(path.x.size(), path.phi.size());
    for(int i = 0; i < path.x.size(); ++i)
    {
        geometry_msgs::Point a;
        a.x = path.x[i];
        a.y = path.y[i];
        a.z = 0;
        m.points.push_back(a);
    }

    paths.markers.push_back(m);
    path_pub_.publish(paths);
}

}
}