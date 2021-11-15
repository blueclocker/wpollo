/*
 * @Author: your name
 * @Date: 2021-11-14 21:52:09
 * @LastEditTime: 2021-11-15 16:28:43
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/lidar/src/euclidean.cpp
 */
#include "lidar/euclidean.h"

euclideancore::euclideancore(ros::NodeHandle n)
{
    seg_distance = {15, 30, 45, 60};
    cluster_distance = {0.5, 1.0, 1.5, 2.0, 2.5};
    sub = n.subscribe("/velodyne_points", 10, &euclideancore::callback, this);
    pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("detected_bounding_boxs", 5);
    pub_noground = n.advertise<sensor_msgs::PointCloud2>("point_no_ground", 5);
    pub_ground = n.advertise<sensor_msgs::PointCloud2>("point_ground", 5);
    ros::spin();
}

euclideancore::~euclideancore()
{
    //
}

void euclideancore::ground_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_in, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_out, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_out_no)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(ground_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.3);
    pass.setNegative(true);
    pass.filter(*ground_out);
    pass.setNegative(false);
    pass.filter(*ground_out_no);
}

void euclideancore::voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr filter_in, pcl::PointCloud<pcl::PointXYZ>::Ptr filter_out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(filter_in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*filter_out);
}

void euclideancore::cluster_bydistance(pcl::PointCloud<pcl::PointXYZ>::Ptr distance_in, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &distance_out)
{
    //distance_out.resize(5);
    for(int i = 0; i < seg_distance.size()+1; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        distance_out[i] = tmp;
    }

    for(size_t i = 0; i < distance_in->size(); ++i)
    {
        pcl::PointXYZ temp;
        temp = distance_in->points[i];

        double raw_distance = std::sqrt(std::pow(temp.x, 2) + std::pow(temp.y, 2));

        if(raw_distance < seg_distance[0])
        {
            distance_out[0]->points.push_back(temp);
        }
        else if(raw_distance < seg_distance[1]){
            distance_out[1]->points.push_back(temp);
        }
        else if(raw_distance < seg_distance[2]){
            distance_out[2]->points.push_back(temp);
        }
        else if(raw_distance < seg_distance[3]){
            distance_out[3]->points.push_back(temp);
        }
        else if(raw_distance < 120){
            distance_out[4]->points.push_back(temp);
        }
        else{
            continue;
        }
    }
}

void euclideancore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr segment_in, 
                        double segment_distance, boost::shared_ptr<std::vector<jsk_recognition_msgs::BoundingBox>> segment_out)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*segment_in, *cloud_2d);
    for(size_t i = 0; i < cloud_2d->size(); ++i)
    {
        cloud_2d->points[i].z = 0;
    }

    if(cloud_2d->points.size() > 0)
    {
        tree->setInputCloud(cloud_2d);
    }

    //聚类器定义和设置
    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(segment_distance);
    euclid.setMaxClusterSize(5000);
    euclid.setMinClusterSize(20);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    for(size_t i = 0; i < local_indices.size(); i++)
    {
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        pcl::PointXYZ center;
        for(auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            pcl::PointXYZ p;
            p = segment_in->points[*pit];
            center.x += p.x;
            center.y += p.y;
            center.z += p.z;

            if(p.x < min_x) min_x = p.x;
            if(p.x > max_x) max_x = p.x;
            if(p.y < min_y) min_y = p.y;
            if(p.y > max_y) max_y = p.y;
            if(p.z < min_z) min_z = p.z;
            if(p.z > max_z) max_z = p.z;
        }

        pcl::PointXYZ maxpoint, minpoint;
        maxpoint.x = max_x;
        maxpoint.y = max_y;
        maxpoint.z = max_z;
        minpoint.x = min_x;
        minpoint.y = min_y;
        minpoint.z = min_z;

        if(local_indices[i].indices.size() > 0)
        {
            center.x /= local_indices[i].indices.size();
            center.y /= local_indices[i].indices.size();
            center.z /= local_indices[i].indices.size();
        }

        double length = maxpoint.x - minpoint.x;
        double width = maxpoint.y - minpoint.y;
        double height = maxpoint.z - minpoint.z;

        jsk_recognition_msgs::BoundingBox bbox;
        //ROS_INFO("jsk_recognition_msgs::BoundingBox");
        bbox.header = point_cloud_header;

        bbox.pose.position.x = minpoint.x + length/2;
        bbox.pose.position.y = minpoint.y + width/2;
        bbox.pose.position.z = minpoint.z + height/2;

        bbox.dimensions.x = std::fabs(length);
        bbox.dimensions.y = std::fabs(width);
        bbox.dimensions.z = std::fabs(height);

        segment_out->push_back(bbox);
        //ROS_INFO("cluster_result.push_back");
    }
}

void euclideancore::callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr noground_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header = cloud_msg_in->header;
    pcl::fromROSMsg(*cloud_msg_in, *current_ptr);

    ground_seg(current_ptr, ground_ptr, noground_ptr);
    sensor_msgs::PointCloud2 pointwithoutground, pointwithground;
    pcl::toROSMsg(*noground_ptr, pointwithoutground);
    pcl::toROSMsg(*ground_ptr, pointwithground);
    pointwithoutground.header = point_cloud_header;
    pointwithground.header = point_cloud_header;
    pub_noground.publish(pointwithoutground);
    pub_ground.publish(pointwithground);

    voxel_filter(noground_ptr, filter_ptr, LEAFSIZE);
    //ROS_INFO("voxel_filter");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_ground(cluster_distance.size());
    cluster_bydistance(filter_ptr, segment_ground);
    //ROS_INFO("cluster_bydistance");

    boost::shared_ptr<std::vector<jsk_recognition_msgs::BoundingBox>> 
                    cluster_result(new std::vector<jsk_recognition_msgs::BoundingBox>);

    for(int i = 0; i < cluster_distance.size(); ++i)
    {
        cluster_segment(segment_ground[i], cluster_distance[i], cluster_result);
    }

    //ROS_INFO("cluster_segment!");
    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header = point_cloud_header;
    bbox_array.boxes = *cluster_result;
    pub.publish(bbox_array);
    //cluster_result.clear();
}