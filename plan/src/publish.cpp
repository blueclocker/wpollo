/*
 * @Author: your name
 * @Date: 2021-12-01 16:14:13
 * @LastEditTime: 2021-12-01 17:09:11
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/publish.cpp
 */
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publishmapnode");
    ros::NodeHandle nh;
    ros::Rate sleeprate(10);
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/map", 10);

    while(nh.ok())
    {
        geometry_msgs::PointStamped point;
        point.header.stamp = ros::Time::now();
        point.header.frame_id = "map";
        point.point.x = -1.2;
        point.point.y = 1.4;
        point.point.z = 12;
        pub.publish(point);
        std::cout << point.point.x << " " << point.point.y << " " << point.point.z << std::endl;
        ros::spinOnce();
        sleeprate.sleep();
    }
    
    return 0;
}