/*
 * @Author: your name
 * @Date: 2021-11-23 16:07:59
 * @LastEditTime: 2021-11-26 19:58:15
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/include/plan/AStar_ros.h
 */
#ifndef ASTAR_ROS_H_
#define ASTAR_ROS_H_

#include "AStar.h"
#include <ros/ros.h>
#include <time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

class AStar_ros: public AStar
{
private:
    ros::Publisher pathpub;
    ros::Publisher mappub;
    ros::Publisher pointpub;
    ros::NodeHandle n;
    ros::Time currenttime;
    //tf
    tf::TransformBroadcaster broadcaster;
    tf::Transform baselink2map;
    bool flag;
public:
    AStar_ros();
    AStar_ros(Node* start, Node* end, const std::vector<std::vector<int>> &map_, const bool isIgnoreCorner_ = false);
    ~AStar_ros();
    void visual(std::list<Node*> visuallist);
    void visual();
    virtual bool search();//搜索是否有解
    virtual std::list<Node*> solve();//A*输出接口
};






#endif