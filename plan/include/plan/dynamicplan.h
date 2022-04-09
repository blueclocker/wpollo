/*
 * @Author: your name
 * @Date: 2022-04-05 13:53:31
 * @LastEditTime: 2022-04-05 19:19:50
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/plan/include/plan/dynamic.h
 */
#ifndef DYNAMICPLAN_H_
#define DYNAMICPLAN_H_

#include <plan/DStar.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/thread.hpp>

#define pi 3.1415926


class DynamicPlan
{
private:
    ros::NodeHandle nh;
    ros::Subscriber point_sub;
    ros::Subscriber start_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber map_sub;
    ros::Publisher path_pub;
    ros::Publisher map_pub;
    ros::Time currenttime;
    nav_msgs::OccupancyGrid newmap;
    geometry_msgs::Quaternion global_start_orientation;
    geometry_msgs::Quaternion global_goal_orientation;

    //
    DStar *dstar;
    bool flag_start;
    bool flag_end;
    bool flag_map;
    bool flag_newmap;
    std::vector<std::vector<int>> global_map;
    DNode start_node;
    DNode goal_node;
    void point_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void start_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void visual(std::vector<DNode> &path_);
public:
    DynamicPlan(ros::NodeHandle n_);
    ~DynamicPlan();
    void dynamicplanrun();
};


#endif