/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-18 20:57:05
 * @LastEditTime: 2022-09-18 21:11:32
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/perception/pcl_test/include/pcl_test_core.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define CLIP_HEIGHT -1.7          //截取掉低于雷达 1.7
#define MIN_DISTANCE 0.8          // 2.4 截取0.8米内点
#define RADIAL_DIVIDER_ANGLE 0.18 // 0.18
#define SENSOR_HEIGHT 1.985517    // 2.2 cs75  雷达高度   1.9 ev200   ；minev

#define concentric_divider_distance_ 0.02 // 0.1 meters default 0.01
#define min_height_threshold_ 0.5         //相邻点坡度0.05
#define local_max_slope_ 8                // max slope of the ground between points, degree  8
#define general_max_slope_ 5              // max slope of the ground in entire point cloud, degree  5
#define reclass_distance_threshold_ 0.2   // 0.2

class PclTestCore
{

private:
  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_ground_, pub_no_ground_, pub_point_;

  struct PointXYZRTColor
  {
    pcl::PointXYZ point;

    float radius; // cylindric coords on XY Plane
    float theta;  // angle deg on XY plane

    size_t radial_div;     // index of the radial divsion to which this point belongs to
    size_t concentric_div; // index of the concentric division to which this points belongs to

    size_t original_index; // index of this point in the source pointcloud
  };
  typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;

  size_t radial_dividers_num_;
  size_t concentric_dividers_num_;

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  void XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                       PointCloudXYZRTColor &out_organized_points,
                       std::vector<pcl::PointIndices> &out_radial_divided_indices,
                       std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds);

  void classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices,
                   pcl::PointIndices &out_no_ground_indices);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);

public:
  PclTestCore(ros::NodeHandle &nh);
  ~PclTestCore();
  void Spin();
};
