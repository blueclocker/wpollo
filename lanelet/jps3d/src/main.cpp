/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-11 20:35:12
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-12 13:54:06
 * @FilePath: /wpollo/src/lanelet/jps3d/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_collision/map_util.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <timer.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

using namespace JPS;
bool start_state_rcv = false, end_state_rcv = false, map_state_rcv = false, has_map = false;
Vec2f start_state, goal_state;
nav_msgs::OccupancyGrid map;
std::shared_ptr<OccMapUtil> map_util;
vec_Vec2f path_jps, path_astar, path_dist;
ros::Publisher jpspath_pub, astarpath_pub;

void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_state.x() = start->pose.pose.position.x;
    start_state.y() = start->pose.pose.position.y;
    // start_state.z() = tf::getYaw(start->pose.pose.orientation);
    start_state_rcv = true;
    std::cout << "get initial state." << std::endl;
}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    goal_state.x() = goal->pose.position.x;
    goal_state.y() = goal->pose.position.y;
    // goal_state.z() = tf::getYaw(goal->pose.orientation);
    end_state_rcv = true;
    std::cout << "get the goal." << std::endl;
}

void mapCb(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    map_state_rcv = true;
    map = *costmap_msg_ptr;
    Vec2f origin_(costmap_msg_ptr->info.origin.position.x, costmap_msg_ptr->info.origin.position.y);
    Vec2i dim_(costmap_msg_ptr->info.width, costmap_msg_ptr->info.height);
    map_util->setMap(origin_, dim_, costmap_msg_ptr->data, costmap_msg_ptr->info.resolution);
    std::cout << "get the map." << std::endl;
}

void PublishjpsPath(const vec_Vec2f &path) 
{
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time::now();

    jpspath_pub.publish(nav_path);
}

void PublishastarPath(const vec_Vec2f &path) 
{
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time::now();

    astarpath_pub.publish(nav_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jps3d");
    ros::NodeHandle nh("~");

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCb);

    jpspath_pub = nh.advertise<nav_msgs::Path>("/jps_path", 1);
    astarpath_pub = nh.advertise<nav_msgs::Path>("/astar_path", 1);
    map_util = std::make_shared<OccMapUtil>();
    // std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>(); //三维

    std::unique_ptr<JPSPlanner2D> planner_ptr(new JPSPlanner2D(false));
    // std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); //三维
    std::unique_ptr<DMPlanner2D> dmp(new DMPlanner2D(false));//边界膨胀
    dmp->setPotentialRadius(Vec2f(1.0, 1.0));
    dmp->setSearchRadius(Vec2f(0.5, 0.5));
    dmp->setMap(map_util, start_state);

    while(nh.ok())
    {
        if (has_map) 
        {
            PublishjpsPath(path_jps);
            PublishastarPath(path_astar);
            continue;
        }
        if (start_state_rcv && end_state_rcv && map_state_rcv)
        {
            std::cout << "searching path" << std::endl;
            //set collision function
            planner_ptr->setMapUtil(map_util);
            //update map
            planner_ptr->updateMap();

            Timer time_jps(true);
            //plan from start to goal using JPS
            bool valid_jps = planner_ptr->plan(start_state, goal_state, true);
            // bool valid_jps = planner_ptr->plan(start_state, goal_state, 1, true); //三维
            double dt_jps = time_jps.Elapsed().count();
            //get the planned raw path from JPS
            path_jps = planner_ptr->getRawPath();
            std::cout << "JPS takes " << dt_jps << " ms." << std::endl;
            std::cout << "JPS openlist and closelist member is " << planner_ptr->getAllSet().size() << std::endl;

            Timer time_astar(true);
            //plan from start to goal using A*
            bool vaild_astar = planner_ptr->plan(start_state, goal_state, false);
            double dt_astar = time_astar.Elapsed().count();
            //get the planned path using A*
            path_astar = planner_ptr->getRawPath();
            std::cout << "A* takes " << dt_astar << " ms." << std::endl;
            std::cout << "A* openlist and closelist member is " << planner_ptr->getAllSet().size() << std::endl;
            
            Timer time_dist(true);
            bool vaild_dist = dmp->computePath(start_state, goal_state, path_jps);
            double dt_dist = time_dist.Elapsed().count();
            path_dist = dmp->getPath();
            std::cout << "DMP takes " << dt_dist << " ms." << std::endl;

            has_map = true;
        }
        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    // ros::spin();
    return 0;
}




