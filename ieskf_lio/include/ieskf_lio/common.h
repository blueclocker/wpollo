/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-08-28 19:23:14
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-10-13 10:45:50
 */
#pragma once
#include <ros/ros.h>
#include "parameters.h"
#include "timer.h"
#include <cloud_msgs/cloud_info.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <execution>
#include <signal.h>
using namespace std;

struct RsPointXYZIRT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

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

using PointXYZIRT = RsPointXYZIRT;
using PointCloudXYZIRT = pcl::PointCloud<PointXYZIRT>;
using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

struct PointXYZIQT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float qx;
    float qy;
    float qz;
    float qw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIQT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, qx, qx)(float, qy, qy)(float, qz, qz)(float, qw, qw)(double, time, time))
typedef PointXYZIQT myPointTypePose;
struct imuData
{
    double timestamp;
    Eigen::Vector3f acc;
    Eigen::Vector3f gyr;
    imuData()
    {
        timestamp=0;
        acc=Eigen::Vector3f::Zero();
        gyr=Eigen::Vector3f::Zero();
    }
    imuData(const sensor_msgs::Imu &msg)
    {
        set(msg);
    }
    void set(const sensor_msgs::Imu &msg)
    {
        timestamp = msg.header.stamp.toSec();
        acc = Eigen::Vector3f(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        gyr = Eigen::Vector3f(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    }
    void set(double &time,Eigen::Vector3f &&acc_,Eigen::Vector3f &&gyr_)
    {
        timestamp=time;
        acc=acc_;
        gyr=gyr_;
    }
};

inline sensor_msgs::PointCloud2 transCloudToRosMsg(PointCloud::Ptr &cloud)
{
    sensor_msgs::PointCloud2 tmpMsg;
    pcl::toROSMsg(*cloud, tmpMsg);
    return tmpMsg;
}

inline PointCloud::Ptr transRosMsg2Cloud(const sensor_msgs::PointCloud2 &msg)
{
    PointCloud::Ptr tmpCloud(new PointCloud());
    pcl::fromROSMsg(msg, *tmpCloud);
    return tmpCloud;
}

inline void publishCloud(ros::Publisher &pub, PointCloud::Ptr &cloud, std_msgs::Header &header)
{
    sensor_msgs::PointCloud2 tmpMsg;
    pcl::toROSMsg(*cloud, tmpMsg);
    tmpMsg.header = header;
    pub.publish(tmpMsg);
}

inline void publishCloud(ros::Publisher &pub, PointCloud::Ptr &cloud, double time, const string &frame)
{
    sensor_msgs::PointCloud2 tmpMsg;
    pcl::toROSMsg(*cloud, tmpMsg);
    tmpMsg.header.stamp = ros::Time().fromSec(time);
    tmpMsg.header.frame_id = frame;
    pub.publish(tmpMsg);
}

inline float rad2deg(float radians) { return radians * 180.0 / M_PI; }

inline float deg2rad(float degrees) { return degrees * M_PI / 180.0; }

inline Eigen::Matrix3f anti_symmetric(Eigen::Vector3f const &_v)
{
    Eigen::Matrix3f _m;
    _m(0, 0) = 0.0;
    _m(0, 1) = -_v.z();
    _m(0, 2) = _v.y();
    _m(1, 0) = _v.z();
    _m(1, 1) = 0.0;
    _m(1, 2) = -_v.x();
    _m(2, 0) = -_v.y();
    _m(2, 1) = _v.x();
    _m(2, 2) = 0.0;
    return _m;
}

inline void anti_symmetric(Eigen::Vector3f const &_v, Eigen::Matrix3f &_m)
{
    _m(0, 0) = 0.0;
    _m(0, 1) = -_v.z();
    _m(0, 2) = _v.y();
    _m(1, 0) = _v.z();
    _m(1, 1) = 0.0;
    _m(1, 2) = -_v.x();
    _m(2, 0) = -_v.y();
    _m(2, 1) = _v.x();
    _m(2, 2) = 0.0;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> GetSkewMatrix(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return w;
}

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
template <typename T>
inline bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold = 0.1f)
{
    if (point.size() < MIN_NUM_MATCH_POINTS)
    {
        return false;
    }

    Eigen::Matrix<T, 3, 1> normvec;

    if (point.size() == NUM_MATCH_POINTS)
    {
        Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        normvec = A.colPivHouseholderQr().solve(b);
    }
    else
    {
        Eigen::MatrixXd A(point.size(), 3);
        Eigen::VectorXd b(point.size(), 1);

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < point.size(); j++)
        {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
        normvec(0, 0) = n(0, 0);
        normvec(1, 0) = n(1, 0);
        normvec(2, 0) = n(2, 0);
    }

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (const auto &p : point)
    {
        Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
        temp[3] = 1.0;
        if (fabs(pca_result.dot(temp)) > threshold)
        {
            return false;
        }
    }
    return true;
}
inline float calc_dist(const PointType &p1, const PointType &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

inline float calc_dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }

PointCloud::Ptr transCloud(PointCloud::Ptr &in, myPointTypePose &pose)
{
    PointCloud::Ptr out(new PointCloud);
    Eigen::Affine3f transCur;
    Eigen::Quaternionf rot{pose.qw, pose.qx, pose.qy, pose.qz};
    transCur.matrix().block<3, 3>(0, 0) = rot.toRotationMatrix();
    transCur.matrix().block<3, 1>(0, 3) = Eigen::Vector3f{pose.x, pose.y, pose.z};
    pcl::transformPointCloud(*in, *out, transCur);
    return out;
}

PointCloud::Ptr transCloud(PointCloud::Ptr &in, Eigen::Quaternionf &rot, Eigen::Vector3f &trans)
{
    PointCloud::Ptr out(new PointCloud);
    Eigen::Affine3f transCur;
    transCur.matrix().block<3, 3>(0, 0) = rot.toRotationMatrix();
    transCur.matrix().block<3, 1>(0, 3) = trans;
    pcl::transformPointCloud(*in, *out, transCur);
    return out;
}

Eigen::Affine3f pclPointToAffine3f(myPointTypePose thisPoint)
{
    Eigen::Affine3f tmp;
    Eigen::Quaternionf rot{thisPoint.qw, thisPoint.qx, thisPoint.qy, thisPoint.qz};
    tmp.linear() = rot.toRotationMatrix();
    tmp.matrix().block<3, 1>(0, 3) = Eigen::Vector3f{thisPoint.x, thisPoint.y, thisPoint.z};
    return tmp;
}

inline std::vector<float> splitPoseLine(std::string _str_line, char _delimiter)
{
    std::vector<float> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter))
    {
        parsed.push_back(std::stof(temp));
    }
    return parsed;
}