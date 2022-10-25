#include <cloud_msgs/cloud_info.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/filters/radius_outlier_removal.h>
//#include "../../../../devel/include/ieskf_lio/SegGroundRANSACConfig.h"
#include "../../include/ieskf_lio/common.h"
#include "../../include/ieskf_lio/parameters.h"
//#include "../../include/segment/segment.hpp"

using namespace std;
using namespace Eigen;

void MySigintHandler(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("preProcess shutting down!");
    ros::shutdown();
}

class Preprocess
{
private:
    std::mutex imuLock;
    std::mutex odomLock;
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Subscriber subImu;
    ros::Subscriber subOdom;

    ros::Publisher pubCloudInfo;
    ros::Publisher pubGround;
    ros::Publisher pubNonGround;
    ros::Publisher pubCornerCloud;
    ros::Publisher pubLessCornerCloud;
    ros::Publisher pubSurfCloud;
    ros::Publisher pubLessSurfCloud;
    PointCloud::Ptr laserCloudIn;
    PointCloud::Ptr validCloud;

    PointCloud::Ptr groundCloud;
    PointCloud::Ptr nongroundCloud;

    cloud_msgs::cloud_info cloudInfo;

    std_msgs::Header cloudHeader;

    int N_SCAN = 32;
    int Horizon_SCAN = 1800;

    std_msgs::Float64MultiArray planeMsg;

    // ground removal
    double sensor_height;
    int groundScanInd;
    double min_delta_height_;
    double local_max_slope;
    double max_delta_radius_;
    double global_max_slope;

    std::deque<sensor_msgs::Imu> imuQueue;
    std::deque<nav_msgs::Odometry> odomQueue;
    double *imuTime = new double[2000];
    float *imuRotX = new float[2000];
    float *imuRotY = new float[2000];
    float *imuRotZ = new float[2000];
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;
    int imuPointerCur;
    double timeScanCur; //第一个点时间
    double timeScanEnd; //最后一个点时间
    double imuRoll, imuPitch, imuYaw;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
    float cloudCurvature[400000];
    int cloudSortInd[400000];
    float cloudRange[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];
    bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

public:
    Preprocess()
    {
        initParameters();

        subLaserCloud = nh.subscribe("/lidar_reorganize", 1, &Preprocess::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &Preprocess::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe("/odometry_imu", 2000, &Preprocess::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        pubCloudInfo = nh.advertise<cloud_msgs::cloud_info>("/cloud_info_odom", 1);
        pubCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/corner_cloud", 1);
        pubLessCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/less_corner_cloud", 1);
        pubSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/surf_cloud", 1);
        pubLessSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/less_surf_cloud", 1);
        pubNonGround = nh.advertise<sensor_msgs::PointCloud2>("/nonGround", 1);
        pubGround = nh.advertise<sensor_msgs::PointCloud2>("/ground", 1);
        ros::spin();
    }

    void initParameters()
    {
        // seg.init();
        laserCloudIn.reset(new PointCloud());
        validCloud.reset(new PointCloud());
        groundCloud.reset(new PointCloud());
        nongroundCloud.reset(new PointCloud());
        for (int i = 0; i < 2000; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
        odomIncreX = 0;
        odomIncreY = 0;
        odomIncreZ = 0;
        firstPointFlag = true;
        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
    }

    void allocateMemory()
    {
        laserCloudIn->clear();
        validCloud->clear();
        groundCloud->clear();
        nongroundCloud->clear();
        planeMsg.data.clear();
        firstPointFlag = true;
        for (int i = 0; i < 2000; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
        odomIncreX = 0;
        odomIncreY = 0;
        odomIncreZ = 0;
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();
    }

    void odometryHandler(const nav_msgs::OdometryPtr &odomMsg)
    {
        nav_msgs::Odometry odom = *odomMsg;
        std::lock_guard<std::mutex> lock1(odomLock);
        odomQueue.emplace_back(odom);
    }

    void imuHandler(const sensor_msgs::ImuConstPtr &imuMsg)
    {
        // 将原始IMU数据通过外参变换转到雷达坐标系下
        sensor_msgs::Imu thisImu = *imuMsg;

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        // *posXCur = 0;
        // *posYCur = 0;
        // *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        float ratio = (relTime - timeScanCur) / (timeScanEnd - timeScanCur);

        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur, posXCur, posYCur, posZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
        findPosition(pointTime, &posXCur, &posYCur, &posZCur);
        // transform points to start
        if (firstPointFlag)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        Eigen::Vector3f newPointVec{point->x, point->y, point->z};
        newPointVec = transBt.rotation() * newPointVec + transBt.translation();
        // newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        // newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        // newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.x = newPointVec.x();
        newPoint.y = newPointVec.y();
        newPoint.z = newPointVec.z();
        newPoint.intensity = point->intensity;
        return newPoint;
    }

    template <typename T>
    void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
    {
        *angular_x = thisImuMsg->angular_velocity.x;
        *angular_y = thisImuMsg->angular_velocity.y;
        *angular_z = thisImuMsg->angular_velocity.z;
    }

    bool odomDeskewInfo()
    {

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return false;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return false;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (startOdomMsg.header.stamp.toSec() < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return false;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (endOdomMsg.header.stamp.toSec() < timeScanEnd)
                continue;
            else
                break;
        }
        cloudInfo.odomGuess = endOdomMsg;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x,
                                                            startOdomMsg.pose.pose.position.y,
                                                            startOdomMsg.pose.pose.position.z,
                                                            roll,
                                                            pitch,
                                                            yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x,
                                                          endOdomMsg.pose.pose.position.y,
                                                          endOdomMsg.pose.pose.position.z,
                                                          roll,
                                                          pitch,
                                                          yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        return true;
    }

    bool imuDeskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return false;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }
        --imuPointerCur;

        if (imuPointerCur <= 0)
            return false;

        return true;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        Timer counter("pre_process");
        copyPointCloud(laserCloudMsg);

        if (imuDeskewInfo() && odomDeskewInfo())
        // if (imuDeskewInfo())
        {
            undistortPointCloud();
        }

        // slopeGroundSegmentation();
        featureExtraction();
        publishTopics();
        allocateMemory();
        counter.tic_toc();
    }

    ~Preprocess() {}

    void undistortPointCloud()
    {
        for (size_t i = 0; i < validCloud->size(); ++i)
        {
            double relTime = validCloud->points[i].intensity - int(validCloud->points[i].intensity);
            validCloud->points[i] = deskewPoint(&validCloud->points[i], relTime);
        }
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        cloudHeader = laserCloudMsg->header;
        timeScanEnd = cloudHeader.stamp.toSec();
        double deltaTime = laserCloudIn->points.back().intensity - int(laserCloudIn->points.back().intensity);
        timeScanCur = timeScanEnd - deltaTime;
        removeCloseFarPoints(laserCloudIn, validCloud);
    }

    void removeCloseFarPoints(const PointCloud::Ptr &in,
                              PointCloud::Ptr &out)
    {
        for (const auto &point : *in)
        {
            if (isnan(point.x) || isnan(point.y) || isnan(point.z))
                continue;
            double range = sqrt(point.x * point.x + point.y * point.y);
            if (range < minSensorRange || range > maxSensorRange)
                continue;
            PointType tmp = point;
            out->push_back(tmp);
        }
    }

    void featureExtraction()
    {

        std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudScansOrigin(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
        {
            laserCloudScansOrigin[i].reset(new pcl::PointCloud<PointType>());
        }

        int cloudSize = validCloud->size();
        vector<int> scanStartInd;
        vector<int> scanEndInd;
        scanStartInd.resize(N_SCAN);
        scanEndInd.resize(N_SCAN);

        for (int i = 0; i < cloudSize; i++)
        {

            int scanID = int(validCloud->points[i].intensity);
            // ROS_INFO("scanID:  %d", scanID);
            if (scanID < 0 || scanID >= N_SCAN)
                continue;
            laserCloudScansOrigin[scanID]->push_back(validCloud->points[i]);
        }

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < N_SCAN; i++)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += *laserCloudScansOrigin[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }

        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

            // cloudRange[i] = std::sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
            //                           laserCloud->points[i].y * laserCloud->points[i].y +
            //                           laserCloud->points[i].z * laserCloud->points[i].z);
            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }

        for (int i = 5; i < cloudSize - 5; ++i)
        {
            float depth1 = cloudRange[i];
            float depth2 = cloudRange[i + 1];
            int columnDiff = std::fabs(int(cloudSortInd[i + 1] - cloudSortInd[i]));
            if (columnDiff < 6)
            {
                // 选择距离较远的那些点，并将他们标记为1(不可选为特征点)
                if (depth1 - depth2 > 0.2)
                {
                    // cloudNeighborPicked[i - 5] = 1;
                    // cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                else if (depth2 - depth1 > 0.2)
                {
                    cloudNeighborPicked[i] = 1;
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    // cloudNeighborPicked[i + 4] = 1;
                    // cloudNeighborPicked[i + 5] = 1;
                    // cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::fabs(cloudRange[i - 1] - cloudRange[i]);
            float diff2 = std::fabs(cloudRange[i + 1] - cloudRange[i]);

            // 选择距离变化较大的点，并将他们标记为1
            if (diff1 > 0.02 * cloudRange[i] && diff2 > 0.02 * cloudRange[i])
                cloudNeighborPicked[i] = 1;
        }
        for (int i = 0; i < N_SCAN; i++)
        {
            if (scanEndInd[i] - scanStartInd[i] < 6)
                continue;
            pcl::PointCloud<PointType>::Ptr lessSurfCloudScan(new pcl::PointCloud<PointType>());
            for (int j = 0; j < 6; j++)
            {
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                auto bound_comp = bind(&Preprocess::comp, this, _1, _2);
                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, bound_comp);

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > 1.0)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        }
                        else
                            break;

                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];

                    if (laserCloud->points[ind].x * laserCloud->points[ind].x + laserCloud->points[ind].y * laserCloud->points[ind].y + laserCloud->points[ind].z * laserCloud->points[ind].z < 0.25)
                        continue;

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < 0.1)
                    {

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                            break;

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                        lessSurfCloudScan->push_back(laserCloud->points[k]);
                }
            }

            pcl::PointCloud<PointType> lessSurfCloudScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(lessSurfCloudScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(lessSurfCloudScanDS);
            *surfPointsLessFlat += lessSurfCloudScanDS;
        }
    }

    void publishTopics()
    {
        publishCloud(pubGround, groundCloud, cloudHeader);
        publishCloud(pubNonGround, nongroundCloud, cloudHeader);
        publishCloud(pubCornerCloud, cornerPointsSharp, cloudHeader);
        publishCloud(pubLessCornerCloud, cornerPointsLessSharp, cloudHeader);
        publishCloud(pubSurfCloud, surfPointsFlat, cloudHeader);
        publishCloud(pubLessSurfCloud, surfPointsLessFlat, cloudHeader);

        // cout << "GroundCloud: " << groundCloud->size() << " points" << endl;
        // cout << "NonGroundCloud: " << nongroundCloud->size() << " points" << endl;

        cloudInfo.header = cloudHeader;
        // cloudInfo.groundCloud = tmpGroundMsg;
        // cloudInfo.segmentedCloud = tmpNonGroundMsg;
        cout << "cornerCloud: " << cornerPointsSharp->size() << " points" << endl;
        cout << "surfCloud: " << surfPointsFlat->size() << " points" << endl;
        cloudInfo.fullCloud = transCloudToRosMsg(validCloud);
        cloudInfo.cornerCloud = transCloudToRosMsg(cornerPointsSharp);
        cloudInfo.lessCornerCloud = transCloudToRosMsg(cornerPointsLessSharp);
        cloudInfo.surfCloud = transCloudToRosMsg(surfPointsFlat);
        cloudInfo.lessSurfCloud = transCloudToRosMsg(surfPointsLessFlat);

        pubCloudInfo.publish(cloudInfo);
        return;
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Preprocess");
    Preprocess process;
    ROS_INFO("\033[1;32m---->\033[0m Preprocess Started.");
    signal(SIGINT, MySigintHandler);
    return 0;
}
