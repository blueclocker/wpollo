/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 19:32:00
 * @LastEditTime: 2022-11-08 22:15:52
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGATION_H_
#define NAVAGATION_H_

#include "osmmap/map_io.h"
#include "osmmap/visualization.h"
#include "osmmap/map_plan.h"
#include "osmmap/dubins.h"
#include "osmmap/cubic_spline.h"
#include "smoother/reference_path_smoother.h"
#include "smoother/smoother.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <fsd_common_msgs/Comb.h>
// #include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <osmmap/Navigation.h>
#include <osmmap/Regulatoryelement.h>
#include <osmmap/CarState.h>
#include <osmmap/Lane.h>
#include <osmmap/Lanes.h>

namespace navagation
{
template <typename T>
T ConstrainAngle(T angle)
{
    if(angle < 180 && angle > 90)
    {
        angle -= 270;
    }else{
        angle += 90;
    }
    return angle;
}

class NavagationBase
{
protected:
    const map::node::Node *nodesptr_;
    const map::way::Way *waysptr_;
    const map::relation::Relation *relationsptr_;
    const map::centerway::CenterWay *centerwaysptr_;
    // const grid_map::GridMap *gridmapsptr_;
    map::Map *vectormap_;
    map::MapVisualization *visualmap_;
    plan::Globalplan *globalplans_;

    //params
    //起点所在路段id
    int start_path_;
    //终点所在路段id
    int end_path_;
    //起点的前一个中心线点id
    int start_centerpoint_id_;
    //终点的下一个中心线点id
    int end_centerpoint_id_;
    //是否存在有效起点
    bool isstart_path_exist_;
    //是否存在有效终点
    bool isend_path_exist_;
    //起点状态, x,y,yaw
    double start_state_[3];
    //终点状态
    double end_state_[3];
    geometry_msgs::Pose endPose_;
    //规划结果，得到的道路中心线id
    std::vector<int> paths_;
    map::node::Point3D *atnowpoint_;//当前点相对坐标
    std::vector<map::centerway::CenterPoint3D> smoothpathnode_;

    //ros
    ros::NodeHandle n_;
    ros::Time currenttime_;
    ros::Subscriber startpoint_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber goalpoint_sub_;

    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher gridmap_pub_;
    ros::Publisher gpspath_pub_;
    ros::Publisher carstate_pub_;
    ros::Publisher navigation_pub_;
    ros::Publisher lanes_pub_;
    ros::Publisher golbalpath_pub_;
    ros::Publisher speed_pub_;

    visualization_msgs::MarkerArray map_markerarray_;
    visualization_msgs::MarkerArray path_markerarray_;
    nav_msgs::OccupancyGrid gridmapmsg_;
    nav_msgs::Path gpspath_;
    osmmap::Navigation laneletinfo_;
    osmmap::Lanes lanesinfo_;

    tf::TransformBroadcaster broadcaster_;
    tf::Transform baselink2map_;

    void FullNavigationInfo();
    void FullLanesInfo(const int id);
    void PushCenterPoint(const std::vector<int> &pathid);
    void OutMapPlan(const map::centerway::CenterPoint3D &atnow_centerpoint, const double heading);
    void SmoothPath();

    void StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
public:
    NavagationBase(ros::NodeHandle &n);
    virtual ~NavagationBase();
    virtual void Process() = 0;
};


}//namespace nagavation

#endif