/*
 * @Author: your name
 * @Date: 2022-04-11 15:20:20
 * @LastEditTime: 2022-04-11 15:20:53
 * @LastEditors: your name
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/test/src/velocity_replanner.h
 */
#ifndef __VELOCITY_REPLANNER_H__
#define __VELOCITY_REPLANNER_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <boost/circular_buffer.hpp>
//#include "autoware_msgs/lane.h"

namespace waypoint_maker
{
class VelocityReplanner
{
private:
  ros::NodeHandle private_nh_;
  double r_th_, r_min_, r_inf_;
  int lookup_crv_width_;
  double velocity_max_, velocity_min_;
  double accel_limit_, decel_limit_, resample_interval_;
  int velocity_offset_;
  bool resample_mode_;
  int end_point_offset_;
  double vel_param_;

public:
  VelocityReplanner();
  ~VelocityReplanner();
  void initParameter(const autoware_msgs::ConfigWaypointLoader::ConstPtr& conf);
  void replanLaneWaypointVel(autoware_msgs::lane* lane);

protected:
  void resampleLaneWaypoint(const double resample_interval, autoware_msgs::lane* lane);
  void resampleOnStraight(const boost::circular_buffer<geometry_msgs::Point>& curve_point, autoware_msgs::lane* lane);
  void resampleOnCurve(const geometry_msgs::Point& target_point, const std::vector<double>& param,
                       autoware_msgs::lane* lane);

  const boost::circular_buffer<geometry_msgs::Point> getCrvPointsOnResample(const autoware_msgs::lane& lane,
                                                                            const autoware_msgs::lane& original_lane,
                                                                            unsigned long original_index) const;
  const boost::circular_buffer<geometry_msgs::Point> getCrvPoints(const autoware_msgs::lane& lane,
                                                                  unsigned long index) const;

  void createRadiusList(const autoware_msgs::lane& lane, std::vector<double>* curve_radius);
  const double calcVelParam() const;
  void createCurveList(const std::vector<double>& curve_radius,
                       std::unordered_map<unsigned long, std::pair<unsigned long, double> >* curve_list);

  void limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vmin,
                            autoware_msgs::lane* lane);
  void limitAccelDecel(const unsigned long idx, autoware_msgs::lane* lane);

  const std::vector<double> calcCurveParam(boost::circular_buffer<geometry_msgs::Point> point) const;
  const double calcPathLength(const autoware_msgs::lane& lane) const;
};
}
#endif
