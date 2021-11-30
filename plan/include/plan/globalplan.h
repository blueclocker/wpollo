/*
 * @Author: your name
 * @Date: 2021-11-28 19:27:44
 * @LastEditTime: 2021-11-30 09:54:02
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/include/plan/globalplan.h
 */
#ifndef GLOBALPLAN_H_
#define GLOBALPLAN_H_

#include <plan/AStar.h>
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

class GlobalPlan
{
private:
    ros::NodeHandle n;
    Node* global_start;
    geometry_msgs::Quaternion global_start_orientation;
    Node* global_end;
    geometry_msgs::Quaternion global_end_orientation;
    std::vector<std::vector<int>> global_map;
    bool flag_start;
    bool flag_end;
    bool flag_map;
    bool flag_search;
    bool flag_finish;
    ros::Subscriber sub_start;
    ros::Subscriber sub_end;
    ros::Subscriber sub_map;
    ros::Publisher pub_path;
    ros::Publisher pub_startpoint;
    ros::Publisher pub_endpoint;
    AStar* astar;
    ros::Time currenttime;
    tf::TransformBroadcaster broadcaster;
    tf::Transform baselink2map;
    void global_start_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void global_end_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void global_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void visual(const std::list<Node*> visual_path);
    void tf_publisher(const double yaw2d_, const tf::Vector3 tf_move);
    void tf_update();
public:
    GlobalPlan(ros::NodeHandle n_);
    ~GlobalPlan();
    std::list<Node*> global_solve();
};






#endif