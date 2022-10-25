/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-09 14:07:52
 * @LastEditTime: 2022-10-09 16:34:27
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parkingonce.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef PARKINGONCE_H_
#define PARKINGONCE_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "open_space/hybrid_a_star.h"
#include "gtest/gtest.h"
#include "math/box2d.h"
#include "math/vec2d.h"
// #include "planning/obstacle.h"
#include "planning/planning_gflags.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace apollo{
namespace planning{
class ParkingOnce
{
public:
    ParkingOnce(ros::NodeHandle &nh);
    ~ParkingOnce();
    void run();
private:
    void setObs();
    void StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void visualizationPath(apollo::planning::HybridAStartResult &path);
    void visualizationObstacles(const std::vector<std::vector<apollo::common::math::Vec2d>> &obstacles_lists);
private:
    ros::Publisher obs_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber startpoint_sub_;
    ros::Subscriber goalpoint_sub_;

    std::vector<std::vector<apollo::common::math::Vec2d>> obstacles_list_;
    double sx_, sy_, sphi_, ex_, ey_, ephi_;
    bool has_obstacle_;
    bool has_start_;
    bool has_end_;
    std::vector<double> XYbounds_;
    apollo::planning::HybridAStartResult result_;
    std::unique_ptr<apollo::planning::HybridAStar> hybrid_test_;
};


}
}

#endif