/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-08-24 14:15:32
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-09-02 20:55:11
 */
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "../include/ieskf_lio/common.h"
#include "../include/ieskf_lio/parameters.h"

void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("lidar_repub shutting down!");
  ros::shutdown();
}

class lidarRepub
{
public:
  ros::Publisher pub_pcl;
  ros::Subscriber sub;
  ros::NodeHandle nh;
  ros::Subscriber imu_calibration_sub;
  std::mutex calibMutex;
  Eigen::Quaternionf imuCalibRot = Eigen::Quaternionf::Identity();
  lidarRepub()
  {
    sub = nh.subscribe(pointCloudTopic, 1, &lidarRepub::LidarCbk, this);
    imu_calibration_sub = nh.subscribe("/imu_calibration", 1, &lidarRepub::imuCalibrationCbk, this);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/lidar_reorganize", 1);
  }

  void imuCalibrationCbk(const std_msgs::Float64MultiArray &imuCalib)
  {
    calibMutex.lock();
    imuCalibRot.w() = (float)imuCalib.data[1];
    imuCalibRot.x() = (float)imuCalib.data[2];
    imuCalibRot.y() = (float)imuCalib.data[3];
    imuCalibRot.z() = (float)imuCalib.data[4];
    calibMutex.unlock();
  }
  void LidarCbk(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    double start, end;
    PointCloud::Ptr outCloud(new PointCloud());
    if (lidarType == RSLIDAR)
    {
      pcl::PointCloud<PointXYZIRT>::Ptr laserIn(new pcl::PointCloud<PointXYZIRT>());

      pcl::fromROSMsg(*msg, *laserIn);
      // double scanTime = laserIn->points.back().timestamp - laserIn->points.front().timestamp;
      start = laserIn->points.front().timestamp;
      end = laserIn->points.back().timestamp;

      for (size_t i = 0; i < laserIn->size(); ++i)
      {
        if (isnan(laserIn->points[i].x) || isnan(laserIn->points[i].y) || isnan(laserIn->points[i].z))
        {
          continue;
        }

        double time = laserIn->points[i].timestamp - start;

        // double s = (time - start) / scanTime;
        PointType point;
        Eigen::Vector3f origPoint{laserIn->points[i].x, laserIn->points[i].y, laserIn->points[i].z};
        calibMutex.lock();
        Eigen::Vector3f basePoint = imuCalibRot * (extQuat * origPoint + extTrans);
        calibMutex.unlock();
        // point.x = laserIn->points[i].x;
        // point.y = laserIn->points[i].y;
        // point.z = laserIn->points[i].z;
        point.x = basePoint.x();
        point.y = basePoint.y();
        point.z = basePoint.z();

        point.intensity = float(laserIn->points[i].ring + time);
        outCloud->push_back(point);
      }
    }
    if (lidarType == VELODYNE)
    {
      pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserIn(new pcl::PointCloud<VelodynePointXYZIRT>());

      pcl::fromROSMsg(*msg, *laserIn);
      // double scanTime = laserIn->points.back().timestamp - laserIn->points.front().timestamp;
      start = laserIn->points.front().time;
      end = laserIn->points.back().time + msg->header.stamp.toSec();

      for (size_t i = 0; i < laserIn->size(); ++i)
      {
        if (isnan(laserIn->points[i].x) || isnan(laserIn->points[i].y) || isnan(laserIn->points[i].z))
        {
          continue;
        }

        double time = laserIn->points[i].time - start;

        // double s = (time - start) / scanTime;
        PointType point;
        Eigen::Vector3f origPoint{laserIn->points[i].x, laserIn->points[i].y, laserIn->points[i].z};
        calibMutex.lock();
        Eigen::Vector3f basePoint = imuCalibRot * (extQuat * origPoint + extTrans);
        calibMutex.unlock();
        // point.x = laserIn->points[i].x;
        // point.y = laserIn->points[i].y;
        // point.z = laserIn->points[i].z;
        point.x = basePoint.x();
        point.y = basePoint.y();
        point.z = basePoint.z();

        point.intensity = float(laserIn->points[i].ring + time);
        outCloud->push_back(point);
      }
    }

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*outCloud, outMsg);
    outMsg.header = msg->header;
    outMsg.header.frame_id = imuFrame;
    outMsg.header.stamp = ros::Time().fromSec(end);
    pub_pcl.publish(outMsg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_repub");
  ros::NodeHandle nh;
  ROS_INFO("\033[1;32m----> start lidarRepub. \033[0m");
  lidarRepub lidar_repub;
  signal(SIGINT, MySigintHandler);
  ros::spin();
  return 0;
}
