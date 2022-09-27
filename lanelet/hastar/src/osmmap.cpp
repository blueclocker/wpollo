/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-12 21:42:34
 * @LastEditTime: 2022-09-13 22:38:40
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/hastar/src/osmmap.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "hastar/hybidacore.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <osmmap/CarState.h>
#include <mutex>

HybidA::Vec3d start_state, goal_state;
bool start_state_rcv = false, end_state_rcv = false, map_state_rcv = false;
ros::Publisher path_pub;
std::shared_ptr<HybidA::hybidacore> kinodynamic_astar_searcher_ptr_;
std::mutex map_mutex, plan_mutex;

void PublishPath(const VectorVec3d &path) 
{
    // nav_msgs::Path nav_path;

    // geometry_msgs::PoseStamped pose_stamped;
    // for (const auto &pose: path) {
    //     pose_stamped.header.frame_id = "map";
    //     pose_stamped.pose.position.x = pose.x();
    //     pose_stamped.pose.position.y = pose.y();
    //     pose_stamped.pose.position.z = 0.0;
    //     pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

    //     nav_path.poses.emplace_back(pose_stamped);
    // }

    // nav_path.header.frame_id = "map";
    // nav_path.header.stamp = ros::Time::now();

    // path_pub.publish(nav_path);
    visualization_msgs::Marker vehicle;
    int index = 0;

    vehicle.header.frame_id = "map";
    vehicle.header.stamp = ros::Time::now();
    vehicle.ns = "path";
    vehicle.action = visualization_msgs::Marker::ADD;
    vehicle.type = visualization_msgs::Marker::LINE_STRIP;
    vehicle.id = index++;
    vehicle.scale.x = 0.5;
    // vehicle.scale.z = 0.01;
    vehicle.color.a = 1;

    vehicle.color.r = 1.0;
    vehicle.color.b = 0.0;
    vehicle.color.g = 0.0;
    vehicle.lifetime = ros::Duration(0.5);
    vehicle.pose.orientation.w = 1;
    for (const auto &pose: path)
    {
        geometry_msgs::Point a;
        a.x = pose.x();
        a.y = pose.y();
        a.z = 0.0;
        vehicle.points.push_back(a);
    }
    path_pub.publish(vehicle);
}

void carstatecallback(const osmmap::CarState::ConstPtr &msgptr)
{
    std::cout << "carstate" << std::endl;

    int x = msgptr->carPose.position.x;
    int y = msgptr->carPose.position.y;
    start_state.x() = x;
    start_state.y() = y;
    geometry_msgs::Quaternion q1 = msgptr->carPose.orientation;
    double siny_cosp = +2.0 * (q1.w * q1.z + q1.x * q1.y);
    double cosy_cosp = +1.0 - 2.0 * (q1.y * q1.y + q1.z * q1.z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    start_state.z() = yaw;
    start_state_rcv = true;

    goal_state.x() = msgptr->endPose.position.x;
    goal_state.y() = msgptr->endPose.position.y;
    goal_state.z() = tf::getYaw(msgptr->endPose.orientation);
    std::cout << "start yaw: " << yaw << ", goal yaw: " << goal_state.z() << std::endl;
    if(goal_state.x() != 0 && goal_state.y() != 0)
    {
        end_state_rcv = true;
    }
    if (start_state_rcv && end_state_rcv && map_state_rcv)
    {
      plan_mutex.lock();
      if(kinodynamic_astar_searcher_ptr_->search(start_state, goal_state))
      {
        std::cout << "find" << std::endl;
        auto path = kinodynamic_astar_searcher_ptr_->getPath();
        PublishPath(path);
      }else{
          std::cout << "not find" << std::endl;
      }
      kinodynamic_astar_searcher_ptr_->reset();
      plan_mutex.unlock();
    }
}

void mapCb(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    if(map_state_rcv) return;
    std::cout << "map" << std::endl;
    // map_mutex.lock();
    map_state_rcv = true;
    double map_resolution = 1.0;
    kinodynamic_astar_searcher_ptr_->init(*costmap_msg_ptr, map_resolution); 
    // map_mutex.unlock();
    std::cout << "get the map." << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking");
    ros::NodeHandle nh("~");

    double steering_angle = nh.param("steering_angle", 15);
    int steering_angle_discrete_num = nh.param("steering_angle_discrete_num", 1);
    double wheel_base = nh.param("wheel_base", 2.0);
    double segment_length = nh.param("segment_length", 2.6);//1.6
    int segment_length_discrete_num = nh.param("segment_length_discrete_num", 8);
    double steering_penalty = nh.param("steering_penalty", 1.5);
    double steering_change_penalty = nh.param("steering_change_penalty", 3.0);
    double reversing_penalty = nh.param("reversing_penalty", 2.0);
    double shot_distance = nh.param("shot_distance", 5.0);
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybidA::hybidacore>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );

    // path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    path_pub = nh.advertise<visualization_msgs::Marker>("path", 1);

    ros::Subscriber map_sub = nh.subscribe("/pcdtogm/grid_map", 1, mapCb);
    ros::Subscriber followmap = nh.subscribe("/mapio/carstate_info", 1, &carstatecallback);

    ros::spin();
    return 0;
}
