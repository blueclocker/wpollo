/*
 * @Author: your name
 * @Date: 2021-11-14 21:52:58
 * @LastEditTime: 2021-11-15 15:53:04
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/lidar/include/euclidean.h
 */
#ifndef EUCLIDEAN_H_
#define EUCLIDEAN_H_

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>

#define LEAFSIZE 0.1

class euclideancore
{
private:
    //分割半径
    std::vector<double> seg_distance;
    //聚类半径
    std::vector<double> cluster_distance;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub_noground;
    ros::Publisher pub_ground;
    std_msgs::Header point_cloud_header;
    //std::vector<jsk_recognition_msgs::BoundingBox> cluster_result;
    //点云降采样，体素滤波
    void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr filter_in, pcl::PointCloud<pcl::PointXYZ>::Ptr filter_out, double leaf_size);
    //地面分割
    void ground_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_in, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_out, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_out_no);
    //点云分块
    void cluster_bydistance(pcl::PointCloud<pcl::PointXYZ>::Ptr distance_in, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &distance_out);
    //聚类
    void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr segment_in, double segment_distance, boost::shared_ptr<std::vector<jsk_recognition_msgs::BoundingBox>> segment_out);
    //回调函数
    void callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_in);
public:
    euclideancore(ros::NodeHandle n);
    ~euclideancore();
};



#endif