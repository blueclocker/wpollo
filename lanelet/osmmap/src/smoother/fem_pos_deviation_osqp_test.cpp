/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-23 17:29:10
 * @LastEditTime: 2022-11-23 19:35:35
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/smoother/fem_pos_deviation_osqp_test.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "smoother/fem_pos_deviation_osqp.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <chrono>

ros::Publisher fem_pos_pub;

void callback(const visualization_msgs::Marker::ConstPtr &msg)
{
    // ROS_INFO("get global path");
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(int i = 0; i < msg->points.size(); ++i)
    {
        x.emplace_back(msg->points[i].x);
        y.emplace_back(msg->points[i].y);
        z.emplace_back(msg->points[i].z);
    }
    const auto start_timestamp = std::chrono::system_clock::now();
    plan::FemPosDeviationOsqp solve(x, y, z);
    solve.SetWeightRef({100.0, 1.0, 10.0});
    std::vector<double> opt_x;
    std::vector<double> opt_y;
    std::vector<double> opt_z;
    bool flag = solve.Solve(&opt_x, &opt_y, &opt_z);
    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    std::cout << "fem pos deviation total time is " << diff.count() * 1000.0 << " ms." << std::endl;
    if(!flag) 
    {
        std::cout << "can not solve !!!" << std::endl;
        return;
    }
    
    visualization_msgs::Marker vispath;
    vispath.header.frame_id = "map";
    vispath.header.stamp = ros::Time::now();
    vispath.id = 1;
    vispath.type = visualization_msgs::Marker::LINE_STRIP;
    vispath.action = visualization_msgs::Marker::ADD;
    vispath.color.a = 0.8;
    vispath.color.g = 1.0;
    vispath.scale.x = 1.0;
    vispath.ns = "fempostest";
    vispath.pose.orientation.w = 1.0;
    for(int i = 0; i < opt_x.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = opt_x[i];
        p.y = opt_y[i];
        p.z = opt_z[i];
        vispath.points.emplace_back(p);
    }
    fem_pos_pub.publish(vispath);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fem_pos_deviation_osqp_test");
    ros::NodeHandle nh("~");
    fem_pos_pub = nh.advertise<visualization_msgs::Marker>("/fem_pos_path", 1);
    ros::Subscriber sub = nh.subscribe("/navagation_node/golbalpath_info", 1, callback);

    ros::spin();
    return 0;
}
