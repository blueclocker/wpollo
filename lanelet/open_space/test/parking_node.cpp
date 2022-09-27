/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-18 16:23:01
 * @LastEditTime: 2022-09-21 14:31:10
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parking_node.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <iostream>
#include "open_space/hybrid_a_star.h"

#include "gtest/gtest.h"
#include "math/box2d.h"
#include "math/vec2d.h"
// #include "planning/obstacle.h"
#include "planning/planning_gflags.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher obs_pub;
ros::Publisher path_pub;

std::vector<std::vector<apollo::common::math::Vec2d>> obstacles_list;
bool has_obstacle_ = false;
std::vector<double> XYbounds_;
apollo::planning::HybridAStartResult result;
std::unique_ptr<apollo::planning::HybridAStar> hybrid_test;

void visualizationObstacles(const std::vector<std::vector<apollo::common::math::Vec2d>> &obstacles_lists)
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
    obs_pub.publish(obses);
}

void visualizationPath(apollo::planning::HybridAStartResult &path)
{
    visualization_msgs::MarkerArray paths;
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.ns = "path";
    m.id = 1;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    m.color.g = 1.0;
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
    path_pub.publish(paths);
}

void obscallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    // if(has_obstacle_) return;
    ROS_INFO("get obstacles!");
    obstacles_list.clear();
    for(int i = 0; i < msg->markers.size(); ++i)
    {
        std::vector<apollo::common::math::Vec2d> a_obstacle;
        for(int j = 0; j < msg->markers[i].points.size(); ++j)
        {
            apollo::common::math::Vec2d point;
            point.set_x(msg->markers[i].points[j].x);
            point.set_y(msg->markers[i].points[j].y);
            // point.x() = msg->markers[i].points[j].x;
            // point.y() = msg->markers[i].points[j].y;
            a_obstacle.emplace_back(point);
        }
        obstacles_list.emplace_back(a_obstacle);
    }
    // has_obstacle_ = true;

    double sx = -15.0;
    double sy = 0.0;
    double sphi = 0.0;
    double ex = 15.0;
    double ey = 0.0;
    double ephi = M_PI/2;
    result.clear();
    hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_list, &result);
    visualizationPath(result);
    visualizationObstacles(obstacles_list);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking");
    ros::NodeHandle nh;
    obs_pub = nh.advertise<visualization_msgs::MarkerArray>("obsmarkers", 1);
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("path", 1);

    // ros::Subscriber obs_sub = nh.subscribe("/adaptive_clustering/markers_tubao", 1, obscallback);

    apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
    // std::unique_ptr<apollo::planning::HybridAStar> hybrid_test;
    hybrid_test = std::unique_ptr<apollo::planning::HybridAStar>(
        new apollo::planning::HybridAStar(planner_open_space_config_));

    double sx = -15.0;
    double sy = 0.0;
    double sphi = 0.0;
    double ex = 15.0;
    double ey = 0.0;
    double ephi = M_PI/2;
    // std::vector<std::vector<apollo::common::math::Vec2d>> obstacles_list;
    // apollo::planning::HybridAStartResult result;
    apollo::common::math::Vec2d obstacle_vertice_a(1.0, 0.0);
    apollo::common::math::Vec2d obstacle_vertice_b(-1.0, 0.0);
    apollo::common::math::Vec2d obstacle_vertice_c(0.0, 1.0);
    apollo::common::math::Vec2d obstacle_vertice_d(0.0, -1.0);
    apollo::common::math::Vec2d obstacle_vertice_e(1.0, -1.0);
    apollo::common::math::Vec2d obstacle_vertice_f(-1.0, -1.0);
    std::vector<apollo::common::math::Vec2d> obstacle_a = {obstacle_vertice_a, obstacle_vertice_b, obstacle_vertice_f, obstacle_vertice_a};
    std::vector<apollo::common::math::Vec2d> obstacle_b = {obstacle_vertice_c, obstacle_vertice_b, obstacle_vertice_f, obstacle_vertice_c};
    // load xy boundary into the Plan() from configuration(Independent from frame)
    // std::vector<double> XYbounds_;
    XYbounds_.push_back(-50.0);//xmin
    XYbounds_.push_back(50.0);//xmax
    XYbounds_.push_back(-50.0);//ymin
    XYbounds_.push_back(50.0);//ymax

    obstacles_list.emplace_back(obstacle_a);
    obstacles_list.emplace_back(obstacle_b);
    //hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_list, &result);

    while(nh.ok())
    {
        visualizationObstacles(obstacles_list);
        hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_list, &result);
        visualizationPath(result);
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    return 0;
}