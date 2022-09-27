/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-07 16:20:08
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-13 14:29:30
 * @FilePath: /wpollo/src/lanelet/hastar/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "hastar/hybidacore.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

HybidA::HAstate::ptr start_state = nullptr, goal_state = nullptr;
nav_msgs::OccupancyGridConstPtr map;
bool start_state_rcv = false, end_state_rcv = false, map_state_rcv = false, has_map = false;
ros::Publisher path_pub_, vehicle_path_pub_, searched_tree_pub_;
std::shared_ptr<HybidA::hybidacore> kinodynamic_astar_searcher_ptr_;

void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    HybidA::Vec3i p;
    p << 0, 0, 0;
    if(start_state == nullptr) start_state = new HybidA::HAstate(p);
    start_state->pos.x() = start->pose.pose.position.x;
    start_state->pos.y() = start->pose.pose.position.y;
    start_state->pos.z() = tf::getYaw(start->pose.pose.orientation);
    start_state_rcv = true;
    std::cout << "get initial state." << std::endl;
}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    HybidA::Vec3i p;
    p << 0, 0, 0;
    if(goal_state == nullptr) goal_state = new HybidA::HAstate(p);
    goal_state->pos.x() = goal->pose.position.x;
    goal_state->pos.y() = goal->pose.position.y;
    goal_state->pos.z() = tf::getYaw(goal->pose.orientation);
    end_state_rcv = true;
    std::cout << "get the goal." << std::endl;
}

void mapCb(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    if(map_state_rcv) return;
    map = costmap_msg_ptr;
    map_state_rcv = true;
    double map_resolution = 0.2;
    kinodynamic_astar_searcher_ptr_->init(*map, map_resolution); 
    std::cout << "get the map." << std::endl;
}

void PublishPath(const VectorVec3d &path) 
{
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();

    path_pub_.publish(nav_path);
}

void PublishVehiclePath(const VectorVec3d &path, double width,
                        double length, unsigned int vehicle_interval = 5u) 
{
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hybida_test");
    ros::NodeHandle nh("~");

    double steering_angle = nh.param("steering_angle", 15);
    int steering_angle_discrete_num = nh.param("steering_angle_discrete_num", 1);
    double wheel_base = nh.param("wheel_base", 2.0);
    double segment_length = nh.param("segment_length", 1.6);
    int segment_length_discrete_num = nh.param("segment_length_discrete_num", 8);
    double steering_penalty = nh.param("steering_penalty", 1.5);
    double steering_change_penalty = nh.param("steering_change_penalty", 3.0);
    double reversing_penalty = nh.param("reversing_penalty", 2.0);
    double shot_distance = nh.param("shot_distance", 5.0);
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybidA::hybidacore>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCb);

    path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/vehicle", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("/searched_tree", 1);

    while(nh.ok())
    {
        if (start_state_rcv && end_state_rcv && map_state_rcv)
        {
            if(kinodynamic_astar_searcher_ptr_->search(start_state->pos, goal_state->pos))
            {
                std::cout << "find" << std::endl;
                auto path = kinodynamic_astar_searcher_ptr_->getPath();
                PublishPath(path);
                PublishVehiclePath(path, 4.0, 2.0, 5u);
                PublishSearchedTree(kinodynamic_astar_searcher_ptr_->getSearchedTree());

                nav_msgs::Path path_ros;
                geometry_msgs::PoseStamped pose_stamped;

                for (const auto &pose: path) 
                {
                    pose_stamped.header.frame_id = "world";
                    pose_stamped.pose.position.x = pose.x();
                    pose_stamped.pose.position.y = pose.y();
                    pose_stamped.pose.position.z = 0.0;

                    pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

                    path_ros.poses.emplace_back(pose_stamped);
                }

                path_ros.header.frame_id = "world";
                path_ros.header.stamp = ros::Time::now();
                static tf::TransformBroadcaster transform_broadcaster;
                for (const auto &pose: path_ros.poses) 
                {
                    tf::Transform transform;
                    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

                    tf::Quaternion q;
                    q.setX(pose.pose.orientation.x);
                    q.setY(pose.pose.orientation.y);
                    q.setZ(pose.pose.orientation.z);
                    q.setW(pose.pose.orientation.w);
                    transform.setRotation(q);

                    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_link"));

                    ros::Duration(0.05).sleep();
                }
            }else{
                std::cout << "not" << std::endl;
            }
            kinodynamic_astar_searcher_ptr_->reset();
            break;
            
        }
        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    // ros::spin();
    delete start_state;
    delete goal_state;
    return 0;
}





