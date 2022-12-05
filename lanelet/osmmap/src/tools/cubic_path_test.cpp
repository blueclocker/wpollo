/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-21 22:03:20
 * @LastEditTime: 2022-11-22 15:21:37
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/tools/cubic_path_test.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "tools/cubic_path.h"
#include "tools/cubic_spline.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <chrono>

ros::Publisher cubicpath_pub;

void callback(const visualization_msgs::Marker::ConstPtr &msg)
{
    ROS_INFO("get global path!");
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
    math::CubicPath cubicpath(x, y, z);
    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    std::cout << "cubic path total time is " << diff.count() * 1000.0 << " ms." << std::endl;
    visualization_msgs::Marker vispath;
    vispath.header.frame_id = "map";
    vispath.header.stamp = ros::Time::now();
    vispath.id = 1;
    vispath.type = visualization_msgs::Marker::LINE_STRIP;
    vispath.action = visualization_msgs::Marker::ADD;
    vispath.color.a = 1.0;
    vispath.color.b = 1.0;
    vispath.scale.x = 0.2;
    vispath.ns = "cubictest";
    vispath.pose.orientation.w = 1.0;
    for(double s = 0.0; s <= 50.0 && s < cubicpath.GetLength(); s += 0.2)
    {
        std::array<double, 3> pathpoint = cubicpath.GetPosition(s);
        geometry_msgs::Point p;
        p.x = pathpoint[0];
        p.y = pathpoint[1];
        p.z = pathpoint[2];
        vispath.points.emplace_back(p);
    }
    cubicpath_pub.publish(vispath);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cubic_path_test");
    ros::NodeHandle nh("~");
    cubicpath_pub = nh.advertise<visualization_msgs::Marker>("cubic_path", 1);
    ros::Subscriber path_sub = nh.subscribe("/navagation_node/golbalpath_info", 1, callback);

    ros::spin();
    return 0;
}

