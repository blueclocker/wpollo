/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-09-15 15:01:59
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-09-16 14:35:42
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct PointXYZIR
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

// velodyne的点云格式
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))
ros::Publisher lidarPub;

void Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<PointXYZIR>::Ptr laserIn(new pcl::PointCloud<PointXYZIR>);
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserOut(new pcl::PointCloud<VelodynePointXYZIRT>);
    pcl::fromROSMsg(*msg, *laserIn);
    VelodynePointXYZIRT veloPoint;
    for (const auto point : *laserIn)
    {
        if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z))
        {
            continue;
        }
        float dist = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if (dist < 1.5)
            continue;
        veloPoint.x = point.x;
        veloPoint.y = point.y;
        veloPoint.z = point.z;
        veloPoint.intensity = point.intensity;
        veloPoint.ring = point.ring;
        veloPoint.time = 0;
        laserOut->push_back(veloPoint);
    }
    laserOut->header = laserIn->header;
    laserOut->is_dense = true;
    ROS_INFO("laserOut:  %ld  points!", laserOut->size());
    sensor_msgs::PointCloud2 outMsg;
    outMsg.header = msg->header;
    // outMsg.header.frame_id = "/cari_lidar";
    pcl::toROSMsg(*laserOut, outMsg);
    lidarPub.publish(outMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cari_lidar_repub");
    ros::NodeHandle nh;
    ros::Subscriber lidarSub = nh.subscribe("/cari_points", 1, Callback);
    lidarPub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
    ros::spin();
    return 0;
}