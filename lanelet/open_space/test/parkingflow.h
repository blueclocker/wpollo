/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-20 21:38:25
 * @LastEditTime: 2022-09-20 22:53:41
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parkingflow.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef PARKINGFLOW_H_
#define PARKINGFLOW_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include "open_space/hybrid_a_star.h"
#include "gtest/gtest.h"
#include "math/box2d.h"
#include "math/vec2d.h"
// #include "planning/obstacle.h"
#include "planning/planning_gflags.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "osmmap/CarState.h"

namespace apollo{
namespace planning{
class ParkingFlow
{
public:
    ParkingFlow(ros::NodeHandle &nh);
    ~ParkingFlow();
    void init();
    void run();
private:
    void obsCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
    void carstateCallback(const osmmap::CarState::ConstPtr &msg);
    void visualizationPath(apollo::planning::HybridAStartResult &path);
    void visualizationObstacles(const std::vector<std::vector<apollo::common::math::Vec2d>> &obstacles_lists);
private:
    ros::Publisher obs_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber obs_sub_;
    ros::Subscriber carstate_sub_;

    std::vector<std::vector<apollo::common::math::Vec2d>> obstacles_list_;
    bool has_obstacle_;
    bool has_start_;
    bool has_end_;
    geometry_msgs::Pose car_pose_;
    std::vector<double> XYbounds_;
    apollo::planning::HybridAStartResult result_;
    std::unique_ptr<apollo::planning::HybridAStar> hybrid_test_;
};




}
}
#endif