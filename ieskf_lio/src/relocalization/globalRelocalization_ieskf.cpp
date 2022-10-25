#include "../../include/ieskf_lio/common.h"
#include "../../include/ieskf_lio/parameters.h"
#include "../../include/ieskf_lio/FilterState.h"
#include "../../include/scanContext/Scancontext.h"
#include "../../include/ivox3d/ivox3d.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <iostream>
#include <ostream>
#include <fstream>
using namespace std;
using namespace faster_lio;
#define POS_ 0
#define ROT_ 3
#define VEL_ 6
#define BIA_ 9
#define BIG_ 12
#define GW_ 15
//#define FORCE_Z_ZERO

class mapOptimization
{
public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using VV4F = std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>;
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    enum SYS_STATUS
    {
        UNINITIAL,
        OTHER_SCAN
    };

    ros::NodeHandle nh;
    SYS_STATUS sysStatus = UNINITIAL;
    // 记录世界坐标系下的pvq
    FilterState filterState;
    // 线程锁
    std::mutex imuLock;
    std::mutex biasLock;

    // ros
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubPath;
    ros::Publisher pubCloudUndisorted;
    ros::Publisher pubMapWorld;
    ros::Subscriber subCloud;
    ros::Subscriber subImuBias;
    ros::Subscriber subGuess;
    cloud_msgs::cloud_info cloudInfo;

    // 保存imu
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;
    std::vector<sensor_msgs::Imu> imuBucket;
    std::vector<sensor_msgs::Imu> imuBucketForCorrect;
    std_msgs::Header laserInfoHeader;
    double timeLaserInfoCur;
    double timeLastProcessing = -1;
    nav_msgs::Path globalPath;
    Eigen::Vector3f accBias;
    Eigen::Vector3f gyrBias;

    // PointCloudXYZIRT::Ptr laserCloudFullLast;
    // PointCloudXYZIRT::Ptr laserCloudFullLastDS;
    // PointCloudXYZIRT::Ptr laserCloudFullLastDS_w;
    // PointCloudXYZIRT::Ptr coeffSel;

    pcl::PointCloud<PointType>::Ptr laserCloudFullLast;
    pcl::PointCloud<PointType>::Ptr laserCloudFullLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudFullLastDS_w;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    pcl::PointCloud<myPointTypePose>::Ptr cloudPoses6D;
    pcl::VoxelGrid<PointType> downSizeFilterScan;

    pcl::PointCloud<PointType>::Ptr firstRegCloud;

    std::vector<PointVector> nearestPoints; // nearest points of current scan
    std::vector<bool> pointSelectedSurf;
    VV4F planeCoef; // plane coeffs
    int laserCloudFullLastDSNum = 0;

    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr; // localmap in ivox

    float transformTobeMapped[18];     // [p_w, q_w, v_w, ba, bq, g_w]
    float transformTobeMappedLast[18]; // [p_w, q_w, v_w, ba, bq, g_w]

    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;

    bool isDegenerate;

    // prediction
    Eigen::Matrix<float, 18, 18> F_t;
    Eigen::Matrix<float, 18, 12> G_t;
    Eigen::Matrix<float, 18, 18> P_t;
    Eigen::Matrix<float, 18, 18> P_t_inv;
    Eigen::Matrix<float, 12, 12> noise_;
    // measurement
    Eigen::Matrix<float, Eigen::Dynamic, 1> residual_;
    Eigen::Matrix<float, Eigen::Dynamic, 18> H_k;
    Eigen::Matrix<float, 18, Eigen::Dynamic> K_k;
    Eigen::Matrix<float, 18, 1> updateVec_;
    Eigen::Matrix<float, 18, 1> errState;
    Eigen::Matrix<float, 18, 18> HRH;
    Eigen::Matrix<float, 18, 18> HRH_inv;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    SCManager scManager;
    vector<myPointTypePose> allGlobalPoses;
    vector<pcl::PointCloud<PointType>::Ptr> allCloudKeyFrames;
    vector<myPointTypePose> SCKeyPoses;
    vector<pcl::PointCloud<PointType>::Ptr> SCRawCloudKeyFrames;
    std::mutex scMutex;
    pcl::PointCloud<PointType>::Ptr cloudGlobalMapDS;
    pcl::PointCloud<PointType>::Ptr cloudScanForInitialize;
    // std::vector<PointCloudXYZIRT::Ptr> saveFrame;
    // ros::ServiceServer srvSaveMap;
    // std::vector<vector<float>> poseVec;

    Eigen::Vector3f initialGuessPose;
    int counter;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyPoses;
    std::string mapPcdPath;
    int sc_fail_count;
    bool is_input_init;
    std::mutex counterMutex;
    imuData lastestImu;

public:
    mapOptimization()
    {
        initParams();
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/mapping_scan", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("/mapping_odometry", 1);
        pubPath = nh.advertise<nav_msgs::Path>("/mapping_path", 1);
        pubCloudUndisorted = nh.advertise<sensor_msgs::PointCloud2>("/mapping_cloud_undisorted", 1);
        pubMapWorld = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 100);
        subCloud = nh.subscribe<cloud_msgs::cloud_info>("/cloud_info_odom", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &mapOptimization::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subGuess = nh.subscribe("/initialpose", 10, &mapOptimization::initialpose_callback, this);
    }
    ~mapOptimization() {}

    void initParams()
    {
        downSizeFilterScan.setLeafSize(0.4, 0.4, 0.4);
        aftMappedTrans.frame_id_ = mapFrame;
        aftMappedTrans.child_frame_id_ = imuFrame;
        ivox_options_.resolution_ = 0.5;
        if (ivox_nearby_type == 0)
        {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
        }
        else if (ivox_nearby_type == 6)
        {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
        }
        else if (ivox_nearby_type == 18)
        {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        }
        else if (ivox_nearby_type == 26)
        {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
        }
        else
        {
            ROS_WARN("unknown ivox_nearby_type, use NEARBY18");
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        }
        ivox_options_.capacity_ = ivox_capacity;
        // localmap init (after LoadParams)
        ivox_ = std::make_shared<IVoxType>(ivox_options_);

        laserCloudFullLast.reset(new PointCloud());
        laserCloudFullLastDS.reset(new PointCloud());
        laserCloudFullLastDS_w.reset(new PointCloud());
        cloudGlobalMapDS.reset(new pcl::PointCloud<PointType>());
        cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());
        cloudPoses6D.reset(new pcl::PointCloud<myPointTypePose>());
        // firstRegCloud.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new PointCloud());
        kdtreeKeyPoses.reset(new pcl::KdTreeFLANN<PointType>);
        for (int i = 0; i < 18; ++i)
        {
            transformTobeMapped[i] = 0;
            transformTobeMappedLast[i] = 0;
        }
        transformTobeMapped[GW_ + 2] = -Gravity;
        transformTobeMappedLast[GW_ + 2] = -Gravity;
        filterState.gn_ = Eigen::Vector3f(0.0, 0.0, -Gravity);
        isDegenerate = false;

        F_t.setZero();
        G_t.setZero();
        P_t.setZero();
        noise_.setZero();
        // asDiagonal()指将向量作为对角线构建对角矩阵
        noise_.block<3, 3>(0, 0) = Eigen::Vector3f(imuAccNoise, imuAccNoise, imuAccNoise).asDiagonal();
        noise_.block<3, 3>(3, 3) = Eigen::Vector3f(imuGyrNoise, imuGyrNoise, imuGyrNoise).asDiagonal();
        noise_.block<3, 3>(6, 6) = Eigen::Vector3f(imuAccBiasN, imuAccBiasN, imuAccBiasN).asDiagonal();
        noise_.block<3, 3>(9, 9) = Eigen::Vector3f(imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).asDiagonal();

        errState.setZero();
        HRH.setZero();
        HRH_inv.setZero();

        accBias.setZero();
        gyrBias.setZero();
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);

        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        mapPcdPath = "/home/wangpeng/map/";
        loadMapPCD();
        counter = 0;
        sc_fail_count = 0;
        is_input_init = false;
    }

    void allocateMemory()
    {

        // // laserCloudFullLast.reset(new PointCloudXYZIRT());
        // laserCloudFullLastDS.reset(new PointCloudXYZIRT());
        // laserCloudFullLastDS_w.reset(new PointCloudXYZIRT());
        //  coeffSel.reset(new PointCloudXYZIRT());

        laserCloudFullLast.reset(new PointCloud());
        laserCloudFullLastDS.reset(new PointCloud());
        laserCloudFullLastDS_w.reset(new PointCloud());
        coeffSel.reset(new PointCloud());
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        // 将原始IMU数据通过外参变换转到雷达坐标系下
        sensor_msgs::Imu thisImu = *imuMsg;

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
    {
        is_input_init = true;
        initialGuessPose = Eigen::Vector3f{pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z};
        // Eigen::Quaternionf quat{pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z};
        // initialGuessPose.block<3, 3>(0, 0) = quat.toRotationMatrix();
        std::cout << GREEN << "Get an initial pose!!!" << RESET << std::endl;
        std::cout << GREEN << "Input initial position: " << initialGuessPose.x() << "  " << initialGuessPose.y() << "  " << initialGuessPose.z() << RESET << std::endl;
        // tf::Quaternion q;
        // tf::quaternionMsgToTF(pose_msg->pose.pose.orientation, q);
        // double initialRoll, initialPitch, initialYaw;
        // tf::Matrix3x3(q).getRPY(initialRoll, initialPitch, initialYaw);
        // std::cout << "Input initial RPY: " << initialRoll << "  " << initialPitch << "  " << initialYaw << std::endl;

        // PointType initPoint;
        // initPoint.x = initialGuessPose.x();
        // initPoint.y = initialGuessPose.y();
        // std::vector<int> indices;
        // std::vector<float> dist;
        // kdtreeKeyPoses->radiusSearch(initPoint, 5.0, indices, dist);
        // if (indices.size() == 0)
        // {
        //     std::cout << "Please enter  true initial pose!!!!!!!!!!" << std::endl;
        //     return;
        // }
        // scMutex.lock();
        // SCKeyPoses.clear();
        // SCRawCloudKeyFrames.clear();
        // scManager.clear();
        // for (int i = 0; i < indices.size(); ++i)
        // {
        //     SCKeyPoses.emplace_back(allGlobalPoses[indices[i]]);
        //     SCRawCloudKeyFrames.emplace_back(allCloudKeyFrames[indices[i]]);
        //     scManager.makeAndSaveScancontextAndKeys(*allCloudKeyFrames[indices[i]]);
        // }
        // sc_fail_count = 0;
        // scMutex.unlock();
    }
    void loadMapPCD()
    {
        std::cout << "Loading Global PCD Map......." << std::endl;
        pcl::PointCloud<PointType>::Ptr cloudGlobalMap(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloudTraj(new pcl::PointCloud<PointType>);
        pcl::io::loadPCDFile(mapPcdPath + "GlobalMap.pcd", *cloudGlobalMap);
        //  pcl::io::loadPCDFile(mapPcdPath + "trajectory.pcd", *cloudTraj);
        //  std::cout << "trajectory size:  " << cloudTraj->size() << std::endl;
        downSizeFilterScan.setInputCloud(cloudGlobalMap);
        downSizeFilterScan.filter(*cloudGlobalMapDS);
        ivox_->AddPoints(cloudGlobalMapDS->points);
        // kdtreeKeyPoses->setInputCloud(cloudTraj);
        std::ifstream pose_file(mapPcdPath + "pose.txt");
        std::string strOneLine;
        int num_poses = 0;

        while (getline(pose_file, strOneLine))
        {
            std::vector<float> ith_pose_vec = splitPoseLine(strOneLine, ' ');
            if (ith_pose_vec.size() == 8)
            {
                myPointTypePose pose;
                pose.x = ith_pose_vec[1];
                pose.y = ith_pose_vec[2];
                pose.z = ith_pose_vec[3];
                pose.qx = ith_pose_vec[4];
                pose.qy = ith_pose_vec[5];
                pose.qz = ith_pose_vec[6];
                pose.qw = ith_pose_vec[7];
                // allGlobalPoses.push_back(pose);
                SCKeyPoses.push_back(pose);
            }
            num_poses++;
        }
        for (int i = 0; i < num_poses; i++)
        {
            stringstream inter;
            string s_temp = std::to_string(i);
            inter << setw(5) << setfill('0') << s_temp;
            inter >> s_temp;

            pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>());
            pcl::io::loadPCDFile<PointType>(mapPcdPath + "RawSubMap/" + s_temp + ".pcd", *points);
            SCRawCloudKeyFrames.push_back(points);
            // allCloudKeyFrames.push_back(points);
            scManager.makeAndSaveScancontextAndKeys(*points);
        }

        std::cout << "Finish loading Global PCD Map." << std::endl;
    }

    void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr &msgIn)
    {

        laserInfoHeader = msgIn->header;
        timeLaserInfoCur = msgIn->header.stamp.toSec();
        // extract info
        cloudInfo = *msgIn;

        pcl::fromROSMsg(msgIn->fullCloud, *laserCloudFullLast);
        // static int count = 0;

        if (sysStatus == UNINITIAL)
        {
            // pcl::fromROSMsg(msgIn->fullCloud, *firstRegCloud);
            removeOldImu();
        }
        else if (sysStatus == OTHER_SCAN)
        {
            Timer t_relocalization("relocalization");
            predictByFilter();
            downsampleCurrentScan();
            updateTransformationByFilter();
            updateFilterState(timeLaserInfoCur - timeLastProcessing);
            updatePath(transformTobeMapped);
            publishOdometry();
            publishFrames();
            t_relocalization.tic_toc();
        }
        counterMutex.lock();
        counter++;
        counterMutex.unlock();
        allocateMemory();

        for (int i = 0; i < 18; i++)
        {
            transformTobeMappedLast[i] = transformTobeMapped[i];
        }
        // std::cout << "********************DEBUG*********************" << std::endl;
        // std::cout << "X Y Z: " << transformTobeMapped[POS_ + 0] << " " << transformTobeMapped[POS_ + 1] << "  " << transformTobeMapped[POS_ + 2] << std::endl;
        // std::cout << "roll pitch yaw: " << transformTobeMapped[ROT_ + 0] << "  " << transformTobeMapped[ROT_ + 1] << "  " << transformTobeMapped[ROT_ + 2] << std::endl;
        // std::cout << "**********************************************" << std::endl
        //           << std::endl;
        timeLastProcessing = timeLaserInfoCur;
    }
    void localizeNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int poseSize = SCKeyPoses.size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= poseSize)
                continue;
            *nearKeyframes += *transCloud(SCRawCloudKeyFrames[keyNear], SCKeyPoses[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        downSizeFilterScan.setInputCloud(nearKeyframes);
        downSizeFilterScan.filter(*nearKeyframes);
    }
    void relocalizationThread()
    {
        while (sysStatus == UNINITIAL)
        {
            counterMutex.lock();
            if (counter % 10 == 0 || sc_fail_count >= 1)
            {
                counterMutex.unlock();
                if (alignInitialPose())
                    sysStatus = OTHER_SCAN;
            }
            else
            {
                counterMutex.unlock();
            }

            // std::chrono::milliseconds dura(2);
            // std::this_thread::sleep_for(dura);
        }
    }

    bool alignInitialPose()
    {
        bool is_aligned = false;
        Timer t_initailization("initizalize pose");
        // cloudScanForInitialize = firstRegCloud;
        cloudScanForInitialize = laserCloudFullLast;
        if (cloudScanForInitialize->points.size() == 0)
            return false;

        std::cout << "the size of incoming lasercloud: " << cloudScanForInitialize->points.size() << std::endl;

        Timer t_SC_localization("SC_localization");
        std::cout << "***Using SC Mode***" << std::endl;

        scManager.makeAndSaveScancontextAndKeys(*cloudScanForInitialize);
        auto detectResult = scManager.detectLoopClosureID();
        int SCKeyPre = detectResult.first;
        float yawdiff = detectResult.second;
        if (SCKeyPre == -1)
        {
            ROS_WARN("Can't find SC Correspondence");
            sc_fail_count++;
            return false;
        }

        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        localizeNearKeyframes(prevKeyframeCloud, SCKeyPre, 3);

        if (cloudScanForInitialize->size() < 300 || prevKeyframeCloud->size() < 1000)
        {
            ROS_WARN("SC PC points too few");
            sc_fail_count++;
            return false;
        }

        Eigen::Affine3f prePose = pclPointToAffine3f(SCKeyPoses[SCKeyPre]);
        Eigen::Matrix4f Tpre2W = prePose.matrix();
        Eigen::Matrix4f Tdyaw = Eigen::Matrix4f::Identity();
        Eigen::AngleAxisf yaw_change(yawdiff, Eigen::Vector3f::UnitZ());
        Tdyaw.block<3, 3>(0, 0) = yaw_change.toRotationMatrix().inverse();
        Eigen::Matrix4f Tcur2W = Eigen::Matrix4f::Identity();
        t_SC_localization.tic_toc();

        Eigen::Matrix4f ndtAlignPose, icpAlignPose;
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        Eigen::Matrix4f alignedPose;
        if (sc_fail_count < 1)
        {
            Tcur2W = Tpre2W * Tdyaw;
            icp.setInputSource(cloudScanForInitialize);
            icp.setInputTarget(cloudGlobalMapDS);
            icp.align(*unused_result, Tcur2W);
            std::cout << "ICP score in initializing process is: " << icp.getFitnessScore() << std::endl;
            icpAlignPose = icp.getFinalTransformation();
            unused_result->clear();
            ndt.setInputSource(cloudScanForInitialize);
            ndt.setInputTarget(cloudGlobalMapDS);
            ndt.align(*unused_result, icpAlignPose);
            ndtAlignPose = ndt.getFinalTransformation();
            std::cout << "NDT score in initializing process is: " << ndt.getFitnessScore() << std::endl;

            // firstRegCloud->clear();
            if (ndt.hasConverged() == false || ndt.getFitnessScore() > historyKeyframeFitnessScore)
            {
                sc_fail_count++;
                std::cout << YELLOW << "Initializing failed,please dont't move!!!" << RESET << std::endl;
                return false;
            }
            else
            {

                alignedPose = ndtAlignPose;
            }
        }
        else
        {

            std::cout << YELLOW << "SC localization failed! Please click rviz to choose initial pose" << RESET << std::endl;
            if (!is_input_init)
            {
                std::cout << YELLOW << "Please input initial pose!!!!" << RESET << std::endl;
                return false;
            }
            Eigen::Matrix4f initGuessPoseMat = Eigen::Matrix4f::Identity();
            initGuessPoseMat.block<3, 1>(0, 3) = initialGuessPose;
            Tcur2W = initGuessPoseMat;
            ndt.setInputSource(cloudScanForInitialize);
            ndt.setInputTarget(cloudGlobalMapDS);
            ndt.align(*unused_result, Tcur2W);
            ndtAlignPose = ndt.getFinalTransformation();
            std::cout << "NDT score in initializing process is: " << ndt.getFitnessScore() << std::endl;
            unused_result->clear();
            icp.setInputSource(cloudScanForInitialize);
            icp.setInputTarget(cloudGlobalMapDS);
            icp.align(*unused_result, ndtAlignPose);
            std::cout << "ICP score in initializing process is: " << icp.getFitnessScore() << std::endl;
            icpAlignPose = icp.getFinalTransformation();

            if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            {
                std::cout << YELLOW << "Initializing failed,please dont't move!!!" << RESET << std::endl;
                return false;
            }
            else
            {
                alignedPose = icpAlignPose;
            }
        }

        std::cout << MAGENTA << "initialized pose is: " << alignedPose << RESET << std::endl;
        Eigen::Quaternionf q{alignedPose.block<3, 3>(0, 0)};
        tf::Quaternion quat;
        quat.setValue(q.x(), q.y(), q.z(), q.w());

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        for (int i = 0; i < 18; ++i)
        {
            transformTobeMapped[i] = 0;
        }
        transformTobeMapped[GW_ + 2] = -Gravity;
        transformTobeMapped[POS_ + 0] = alignedPose(0, 3);
        transformTobeMapped[POS_ + 1] = alignedPose(1, 3);
        transformTobeMapped[POS_ + 2] = alignedPose(2, 3);
        transformTobeMapped[ROT_ + 0] = roll;
        transformTobeMapped[ROT_ + 1] = pitch;
        transformTobeMapped[ROT_ + 2] = yaw;
        for (int i = 0; i < 18; i++)
        {
            transformTobeMappedLast[i] = transformTobeMapped[i];
        }

        std::cout << BLUE << "********************Initial Pose*********************" << std::endl;
        std::cout << "X Y Z: " << transformTobeMapped[POS_ + 0] << " " << transformTobeMapped[POS_ + 1] << "  " << transformTobeMapped[POS_ + 2] << std::endl;
        std::cout << "roll pitch yaw: " << transformTobeMapped[ROT_ + 0] << "  " << transformTobeMapped[ROT_ + 1] << "  " << transformTobeMapped[ROT_ + 2] << std::endl;
        std::cout << "*****************************************" << std::endl
                  << RESET << std::endl;
        t_initailization.tic_toc();
        std::cout << GREEN << "Initializing Succeed" << RESET << std::endl;
        return true;
    }

    void transformPoint(const PointType &pi, PointType &po, const Eigen::Affine3f &transIn)
    {
        po = pi;
        po.x = transIn(0, 0) * pi.x + transIn(0, 1) * pi.y + transIn(0, 2) * pi.z + transIn(0, 3);
        po.y = transIn(1, 0) * pi.x + transIn(1, 1) * pi.y + transIn(1, 2) * pi.z + transIn(1, 3);
        po.z = transIn(2, 0) * pi.x + transIn(2, 1) * pi.y + transIn(2, 2) * pi.z + transIn(2, 3);
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, float *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = trans2Affine3f(transformIn);

        vector<int> index(cloudSize, 0);
        for (int i = 0; i < cloudSize; i++)
            index[i] = i;
        // 采用引用捕获

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloudIn, &cloudOut, &transCur](int i)
                      {
                          PointType &pointFrom = cloudIn->points[i];
                          cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
                          cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
                          cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
                          cloudOut->points[i].intensity = pointFrom.intensity; });

        return cloudOut;
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[POS_ + 0], transformIn[POS_ + 1], transformIn[POS_ + 2], transformIn[ROT_ + 0], transformIn[ROT_ + 1], transformIn[ROT_ + 2]);
    }

    void removeOldImu()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        sensor_msgs::Imu frontImu;
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeLaserInfoCur)
            {
                frontImu = imuQueue.front();
                imuQueue.pop_front();
            }
            else
                break;
        }
        imuQueue.push_front(frontImu); // Imu coverage lidar
    }

    void getImuBucket()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        imuBucket.clear();
        sensor_msgs::Imu thisImu;
        // imu    last|---------------|---------------|cur
        // lidar   last|***************|***************|cur
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeLaserInfoCur)
            {
                thisImu = imuQueue.front();
                imuBucket.emplace_back(thisImu);
                imuQueue.pop_front();
            }
            else
                break;
        }
        imuQueue.push_front(thisImu);
    }

    void downsampleCurrentScan()
    {
        laserCloudFullLastDS->clear();
        for (int i = 0; i < laserCloudFullLast->size(); i++)
            if (i % point_filter_num == 0)
                laserCloudFullLastDS->points.emplace_back(laserCloudFullLast->points[i]);

        // Downsample cloud from current scan
        downSizeFilterScan.setInputCloud(laserCloudFullLastDS);
        downSizeFilterScan.filter(*laserCloudFullLastDS);
        laserCloudFullLastDSNum = laserCloudFullLastDS->size();
    }

    void featureMatching(int iterCount)
    {
        Eigen::Affine3f transCur = trans2Affine3f(transformTobeMapped);
        laserCloudFullLastDS_w->resize(laserCloudFullLastDSNum);
        nearestPoints.resize(laserCloudFullLastDSNum, PointVector());
        pointSelectedSurf.resize(laserCloudFullLastDSNum, false);
        planeCoef.resize(laserCloudFullLastDSNum, Eigen::Vector4f::Zero());
        coeffSel->resize(laserCloudFullLastDSNum);

        int cnt_pts = laserCloudFullLastDSNum;
        vector<size_t> index(cnt_pts);
        for (int i = 0; i < cnt_pts; i++)
            index[i] = i;
        std::mutex m;
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&, this](int i)
                      {
            PointType& pointBody = laserCloudFullLastDS->points[i];
            PointType& pointWorld = laserCloudFullLastDS_w->points[i];
            auto &pointsNear = nearestPoints[i];
            transformPoint(pointBody, pointWorld, transCur);

            ivox_->GetClosestPoint(pointWorld, pointsNear, NUM_MATCH_POINTS);
            pointSelectedSurf[i] = pointsNear.size()>=MIN_NUM_MATCH_POINTS;

            if(pointSelectedSurf[i])
            {
                pointSelectedSurf[i] = esti_plane(planeCoef[i], pointsNear, esti_plane_threshold);
            }
            if(pointSelectedSurf[i])
            {
                // 记录合适的法向量、点(base)、点面距离
                auto temp = pointWorld.getVector4fMap();
                temp[3] = 1.0;
                float pd2 = planeCoef[i].dot(temp);
                float s;
                // s = 1- 0.9 * fabs(pd2)/ sqrt(sqrt(pointBody.x * pointBody.x
                //             + pointBody.y * pointBody.y + pointBody.z * pointBody.z));
                s = sqrt(pointBody.x * pointBody.x
                    + pointBody.y * pointBody.y + pointBody.z * pointBody.z)
                    - 81 * pd2 * pd2;
                if(s > 0.0)
                {
                    pointSelectedSurf[i] = true;
                    // 法向量：V4f planeCoef[i]
                    coeffSel->points[i].x = planeCoef[i][0]; // 恢复未归一化的法向量
                    coeffSel->points[i].y = planeCoef[i][1];
                    coeffSel->points[i].z = planeCoef[i][2];
                    // 原始点：PointType laserCloudFullLastDS->points[i]
                    // 残差
                    coeffSel->points[i].intensity = pd2;
                }
            } });
    }

    void predictByFilter()
    {
        // biasLock.lock();
        // filterState.ba_ = accBias.cast<float>();
        // filterState.bw_ = gyrBias.cast<float>();
        //  biasLock.unlock();
        // imu    last|---------------|---------------|cur
        // lidar   last|***************|***************|cur
        getImuBucket();
        std::cout << "Get " << imuBucket.size() << " IMUs to predict." << std::endl;
        if (imuBucket.size() <= 3)
            return;
        // 将transformTobeMapped转成矩阵形式
        Eigen::Affine3f T_transformTobeMapped = trans2Affine3f(transformTobeMapped);
        // rotation, position, velocity
        Eigen::Quaternionf R_transformTobeMapped(T_transformTobeMapped.rotation());
        Eigen::Vector3f P_transformTobeMapped = T_transformTobeMapped.matrix().block(0, 3, 3, 1);
        std::cout << "Vx Vy Vz: " << transformTobeMapped[VEL_ + 0] << " " << transformTobeMapped[VEL_ + 1] << "  " << transformTobeMapped[VEL_ + 2] << std::endl;
        Eigen::Vector3f V_transformTobeMapped = Eigen::Vector3f(transformTobeMapped[VEL_ + 0],
                                                                transformTobeMapped[VEL_ + 1], transformTobeMapped[VEL_ + 2]);

        Eigen::Vector3f un_acc_last, un_gyr_last, un_acc_next, un_gyr_next;
        imuData thisImu(imuBucket[0]);
        imuData lastImu = thisImu;
        double imuTime_last = thisImu.timestamp;
        double dt = 0.0;
        // convert to w frame (remove the grarity)
        un_acc_last = R_transformTobeMapped * (thisImu.acc - filterState.ba_) + filterState.gn_;
        un_gyr_last = thisImu.gyr - filterState.bw_;
        for (int i = 1; i < imuBucket.size(); i++)
        {
            thisImu.set(imuBucket[i]);
            dt = thisImu.timestamp - imuTime_last;
            un_acc_next = R_transformTobeMapped * (thisImu.acc - filterState.ba_) + filterState.gn_;
            un_gyr_next = thisImu.gyr - filterState.bw_;
            Eigen::Vector3f un_acc = 0.5 * (un_acc_last + un_acc_next); // world frame
            Eigen::Vector3f un_gyr = 0.5 * (un_gyr_last + un_gyr_next);
            // 求角度变化量，再转化成李群
            Eigen::Quaternionf dq = axis2Quat(un_gyr * dt);
            // 更新PVQ
            R_transformTobeMapped = (R_transformTobeMapped * dq).normalized(); // base坐标系的变换，右乘
            V_transformTobeMapped = V_transformTobeMapped + un_acc * dt;
            P_transformTobeMapped = P_transformTobeMapped + V_transformTobeMapped * dt + 0.5 * un_acc * dt * dt;

            // predict relative transformation
            filterState.rn_ = P_transformTobeMapped;
            filterState.vn_ = V_transformTobeMapped;
            filterState.qbn_ = R_transformTobeMapped;
            // filterState.ba_?
            // filterState.bw_? 后续看一下怎么更新比较好，参考预积分的线性化
            // filterState.gn_?

            F_t.setIdentity();
            Eigen::Vector3f midAcc = 0.5 * (thisImu.acc + lastImu.acc);
            Eigen::Vector3f midGyr = 0.5 * (thisImu.gyr + lastImu.gyr);
            F_t.block<3, 3>(POS_, VEL_) = dt * Eigen::Matrix3f::Identity();
            F_t.block<3, 3>(ROT_, ROT_) = Eigen::Matrix3f::Identity() + anti_symmetric(-dt * (midGyr - filterState.bw_));
            F_t.block<3, 3>(ROT_, BIG_) = -dt * Eigen::Matrix3f::Identity();
            F_t.block<3, 3>(VEL_, ROT_) = -dt * filterState.qbn_.toRotationMatrix() * anti_symmetric(midAcc - filterState.ba_);
            F_t.block<3, 3>(VEL_, BIA_) = -dt * filterState.qbn_.toRotationMatrix();
            F_t.block<3, 3>(VEL_, GW_) = dt * Eigen::Matrix3f::Identity();

            G_t.setZero();
            G_t.block<3, 3>(VEL_, 0) = -filterState.qbn_.toRotationMatrix();
            G_t.block<3, 3>(ROT_, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
            G_t.block<3, 3>(BIA_, 6) = Eigen::Matrix<float, 3, 3>::Identity();
            G_t.block<3, 3>(BIG_, 9) = Eigen::Matrix<float, 3, 3>::Identity();

            P_t = F_t * P_t * F_t.transpose() + (dt * G_t) * noise_ * (dt * G_t).transpose();

            imuTime_last = thisImu.timestamp;
            un_acc_last = un_acc_next;
            un_gyr_last = un_gyr_next;
            lastImu = thisImu;
        }
        lastestImu.set(lastImu.timestamp, lastImu.acc - filterState.ba_, thisImu.gyr - filterState.bw_);
        // P_transformTobeMapped.z() = cloudInfo.odomGuess.pose.pose.position.z;
        P_t_inv = P_t.colPivHouseholderQr().inverse();

        // remap to transformTobeMapped
        T_transformTobeMapped.setIdentity();
        T_transformTobeMapped.pretranslate(P_transformTobeMapped);
        T_transformTobeMapped.rotate(R_transformTobeMapped);
        pcl::getTranslationAndEulerAngles(T_transformTobeMapped,
                                          transformTobeMapped[POS_ + 0], transformTobeMapped[POS_ + 1], transformTobeMapped[POS_ + 2],
                                          transformTobeMapped[ROT_ + 0], transformTobeMapped[ROT_ + 1], transformTobeMapped[ROT_ + 2]);
        // 更新速度
        for (int i = 0; i < 3; i++)
            transformTobeMapped[VEL_ + i] = V_transformTobeMapped(i, 0);
    }

    bool updateTransformationIESKF(int iterCount)
    {
        double residualNorm = 1e6;
        bool hasConverged = false;
        bool hasDiverged = false;
        // lidar base
        float srx = sin(transformTobeMapped[ROT_ + 0]);
        float crx = cos(transformTobeMapped[ROT_ + 0]);
        float sry = sin(transformTobeMapped[ROT_ + 1]);
        float cry = cos(transformTobeMapped[ROT_ + 1]);
        float srz = sin(transformTobeMapped[ROT_ + 2]);
        float crz = cos(transformTobeMapped[ROT_ + 2]);

        int laserCloudSelNum = laserCloudFullLastDSNum;
        int VaildCount = 0;
        for (int i = 0; i < laserCloudFullLastDSNum; i++)
        {
            if (pointSelectedSurf[i])
                VaildCount++;
        }
        if (VaildCount < 10)
        {
            return true; // 直接跳出
        }

        residual_ = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(laserCloudSelNum, 1);
        H_k = Eigen::Matrix<float, Eigen::Dynamic, 18>::Zero(laserCloudSelNum, 18);
        // R_K为测量噪声协方差矩阵的逆
        // R_k       = LIDAR_STD*Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(laserCloudSelNum, laserCloudSelNum);
        // R_k_inv   = (1/LIDAR_STD)*Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(laserCloudSelNum, laserCloudSelNum);
        K_k = Eigen::Matrix<float, 18, Eigen::Dynamic>::Zero(18, laserCloudSelNum);
        updateVec_.setZero();

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++)
        {
            if (!pointSelectedSurf[i])
                continue;

            pointOri.x = laserCloudFullLastDS->points[i].x;
            pointOri.y = laserCloudFullLastDS->points[i].y;
            pointOri.z = laserCloudFullLastDS->points[i].z;

            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;
            float &px = pointOri.x;
            float &py = pointOri.y;
            float &pz = pointOri.z;
            float &nx = coeff.x;
            float &ny = coeff.y;
            float &nz = coeff.z;
            float arx = py * (nx * (srx * srz + crx * crz * sry) - ny * (crz * srx - crx * sry * srz) +
                              nz * crx * cry) -
                        pz * (ny * (crx * crz + srx * sry * srz) - nx * (crx * srz - crz * srx * sry) + nz * cry * srx);
            float ary = pz * (nx * crx * cry * crz - nz * crx * sry + ny * crx * cry * srz) + py * (nx * cry * crz * srx - nz * srx * sry + ny * cry * srx * srz) - px * (nz * cry + nx * crz * sry + ny * sry * srz);
            float arz = px * (ny * cry * crz - nx * cry * srz) - py * (nx * (crx * crz + srx * sry * srz) + ny * (crx * srz - crz * srx * sry)) + pz * (nx * (crz * srx - crx * sry * srz) + ny * (srx * srz + crx * crz * sry));

            H_k(i, ROT_ + 0) = arx;
            H_k(i, ROT_ + 1) = ary;
            H_k(i, ROT_ + 2) = arz;
            H_k(i, POS_ + 0) = coeff.x;
            H_k(i, POS_ + 1) = coeff.y;
            H_k(i, POS_ + 2) = coeff.z;
#ifdef FORCE_Z_ZERO
            H_k(i, ROT_ + 0) = 0;
            H_k(i, ROT_ + 1) = 0;
            H_k(i, POS_ + 2) = 0;
#endif
            residual_(i, 0) = optimizationStep * coeff.intensity;
        }

        // 原始
        // HRH_P = (H_k.transpose()*R_k_inv*H_k+P_t_inv).block(0, 0, 18, 18);
        // HRH_P_inv = HRH_P.colPivHouseholderQr().inverse();
        // K_k = HRH_P_inv*H_k.transpose()*R_k_inv;

        // 改进， 省略了大矩阵R的计算，速度会快一个数量级
        Eigen::Matrix<float, 18, 18> P_tmp = LIDAR_STD * P_t_inv;
        HRH = H_k.transpose() * H_k + P_tmp;
        HRH_inv = HRH.colPivHouseholderQr().inverse();
        K_k = HRH_inv * H_k.transpose();

        updateVec_ = K_k * (H_k * errState - residual_) - errState;

        // cout<<K_k<<endl<<endl;

        // Divergence determination
        // 迭代发散判断
        bool hasNaN = false;
        for (int i = 0; i < updateVec_.size(); i++)
        {
            if (isnan(updateVec_[i]))
            {
                updateVec_[i] = 0;
                hasNaN = true;
            }
        }
        if (hasNaN == true)
        {
            ROS_WARN("System diverges Because of NaN...");
            hasDiverged = true;
            return true;
        }
        // Check whether the filter converges
        // 检查滤波器是否迭代收敛
        if (residual_.norm() > residualNorm * 10)
        {
            ROS_WARN("System diverges...");
            hasDiverged = true;
            return true;
        }

        errState += updateVec_;
        // std::cout << "updateVec: " << updateVec_.transpose() << std::endl;

        transformTobeMapped[ROT_ + 0] += updateVec_(ROT_ + 0, 0);
        transformTobeMapped[ROT_ + 1] += updateVec_(ROT_ + 1, 0);
        transformTobeMapped[ROT_ + 2] += updateVec_(ROT_ + 2, 0);
        transformTobeMapped[POS_ + 0] += updateVec_(POS_ + 0, 0);
        transformTobeMapped[POS_ + 1] += updateVec_(POS_ + 1, 0);
        transformTobeMapped[POS_ + 2] += updateVec_(POS_ + 2, 0);
        // transformTobeMapped[VEL_+0] += updateVec_(VEL_+0, 0);
        // transformTobeMapped[VEL_+1] += updateVec_(VEL_+1, 0);
        // transformTobeMapped[VEL_+2] += updateVec_(VEL_+2, 0);

        bool coverage = true;
        for (int i = 0; i < 3; i++)
        {
            if (fabs(updateVec_(ROT_ + i)) > 0.001 || fabs(updateVec_(POS_ + i)) > 0.001)
            {
                coverage = false;
                break;
            }
        }
        return coverage;
    }

    void updateTransformationByFilter()
    {
        for (int iter = 0; iter < 10; iter++)
        {
            featureMatching(iter);
            updateTransformationIESKF(iter);
            // break;
        }
        Eigen::Matrix<float, 18, 18> I_ = Eigen::Matrix<float, 18, 18>::Identity();
        // P_t = (I_-K_k*H_k)*P_t*(I_-K_k*H_k).transpose()+K_k*R_k*K_k.transpose();
        P_t = (I_ - K_k * H_k) * P_t;
        errState.setZero();
    }

    void updateFilterState(double dt)
    {
        for (int i = 0; i < 3; i++)
        {
            // transformTobeMapped[VEL_+i] = (transformTobeMapped[POS_+i]-transformTobeMappedLast[POS_+i])/dt;
            // filterState.vn_(i) = transformTobeMapped[VEL_+i];

            // 看作匀加速运动
            transformTobeMapped[VEL_ + i] = 2.0 * (transformTobeMapped[POS_ + i] - transformTobeMappedLast[POS_ + i]) / dt - transformTobeMappedLast[VEL_ + i];
            filterState.vn_(i) = transformTobeMapped[VEL_ + i];
        }
        // filterState.vn_ = Eigen::Vector3f(transformTobeMapped[VEL_+0], transformTobeMapped[VEL_+1], transformTobeMapped[VEL_+2]);
        // 将transformTobeMapped转成矩阵形式
        Eigen::Affine3f T_transformTobeMapped = trans2Affine3f(transformTobeMapped);
        filterState.rn_ = T_transformTobeMapped.translation();
        filterState.qbn_ = T_transformTobeMapped.rotation();
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    void updatePath(const float pose_in[])
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = cloudInfo.header.stamp;
        pose_stamped.header.frame_id = mapFrame;
        pose_stamped.pose.position.x = pose_in[0];
        pose_stamped.pose.position.y = pose_in[1];
        pose_stamped.pose.position.z = pose_in[2];
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in[3], pose_in[4], pose_in[5]);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void mapIncremental()
    {
        PointVector points_to_add;
        PointVector point_no_need_downsample;

        int cur_pts = laserCloudFullLastDS->size();
        points_to_add.reserve(cur_pts);
        point_no_need_downsample.reserve(cur_pts);

        std::vector<size_t> index(cur_pts);
        for (size_t i = 0; i < cur_pts; ++i)
        {
            index[i] = i;
        }
        /* transform to world frame */
        laserCloudFullLastDS_w = transformPointCloud(laserCloudFullLastDS, transformTobeMapped);
        // saveFrame.push_back(laserCloudFullLastDS_w);
        //  vector<float> thisPose;
        //  for (int i = 0; i < 6; ++i)
        //  {
        //      thisPose.push_back(transformTobeMapped[i]);
        //  }
        //  poseVec.push_back(thisPose);

        std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i)
                      {
            PointType &point_world = laserCloudFullLastDS_w->points[i];

            if(!nearestPoints[i].empty())
            {
                const PointVector &points_near = nearestPoints[i];
                Eigen::Vector3f center =
                    ((point_world.getVector3fMap() / filter_size_map).array().floor() + 0.5) * filter_size_map;
                Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

                if (fabs(dis_2_center.x()) > 0.5 * filter_size_map &&
                    fabs(dis_2_center.y()) > 0.5 * filter_size_map &&
                    fabs(dis_2_center.z()) > 0.5 * filter_size_map) {
                    point_no_need_downsample.emplace_back(point_world);
                    return;
                }

                bool need_add = true;
                float dist = calc_dist(point_world.getVector3fMap(), center);
                if (points_near.size() >= NUM_MATCH_POINTS) {
                    for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                        if (calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                            need_add = false;
                            break;
                        }
                    }
                }
                if (need_add) {
                    points_to_add.emplace_back(point_world);
                }
            }
            else
            {
                points_to_add.emplace_back(point_world);
            } });

        ivox_->AddPoints(points_to_add);
        ivox_->AddPoints(point_no_need_downsample);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = cloudInfo.header.stamp;
        laserOdometryROS.header.frame_id = mapFrame;
        laserOdometryROS.child_frame_id = imuFrame;
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[POS_ + 0];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[POS_ + 1];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[POS_ + 2];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[ROT_ + 0], transformTobeMapped[ROT_ + 1], transformTobeMapped[ROT_ + 2]);

        Eigen::Vector3f mapVelocity{transformTobeMapped[VEL_ + 0], transformTobeMapped[VEL_ + 1], transformTobeMapped[VEL_ + 2]};
        Eigen::Quaternionf quat{laserOdometryROS.pose.pose.orientation.w, laserOdometryROS.pose.pose.orientation.x, laserOdometryROS.pose.pose.orientation.y, laserOdometryROS.pose.pose.orientation.z};
        Eigen::Vector3f baseVelocity = quat.inverse() * mapVelocity;
        laserOdometryROS.twist.twist.linear.x = baseVelocity.x();
        laserOdometryROS.twist.twist.linear.y = baseVelocity.y();
        laserOdometryROS.twist.twist.linear.z = baseVelocity.z();
        laserOdometryROS.twist.twist.angular.x = lastestImu.gyr.x();
        laserOdometryROS.twist.twist.angular.y = lastestImu.gyr.y();
        laserOdometryROS.twist.twist.angular.z = lastestImu.gyr.z();
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        myPointTypePose pose;
        pose.x = transformTobeMapped[POS_ + 0];
        pose.y = transformTobeMapped[POS_ + 1];
        pose.z = transformTobeMapped[POS_ + 2];
        pose.qx = laserOdometryROS.pose.pose.orientation.x;
        pose.qy = laserOdometryROS.pose.pose.orientation.y;
        pose.qz = laserOdometryROS.pose.pose.orientation.z;
        pose.qw = laserOdometryROS.pose.pose.orientation.w;
        pose.time = timeLaserInfoCur;
        cloudPoses6D->push_back(pose);

        std::cout << "********************Odometry*********************" << std::endl;
        std::cout << "X Y Z: " << transformTobeMapped[POS_ + 0] << " " << transformTobeMapped[POS_ + 1] << "  " << transformTobeMapped[POS_ + 2] << std::endl;
        std::cout << "roll pitch yaw: " << transformTobeMapped[ROT_ + 0] << "  " << transformTobeMapped[ROT_ + 1] << "  " << transformTobeMapped[ROT_ + 2] << std::endl;
        std::cout << "*****************************************" << std::endl
                  << std::endl;
        aftMappedTrans.stamp_ = cloudInfo.header.stamp;
        aftMappedTrans.setRotation(tf::createQuaternionFromRPY(transformTobeMapped[ROT_ + 0], transformTobeMapped[ROT_ + 1], transformTobeMapped[ROT_ + 2]));
        aftMappedTrans.setOrigin(tf::Vector3(transformTobeMapped[POS_ + 0], transformTobeMapped[POS_ + 1], transformTobeMapped[POS_ + 2]));
        // tfBroadcaster.sendTransform(aftMappedTrans);
    }

    void publishFrames()
    {
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = cloudInfo.header.stamp;
            globalPath.header.frame_id = mapFrame;
            pubPath.publish(globalPath);
        }
        if (pubLaserCloudSurround.getNumSubscribers() != 0)
        {
            myPointTypePose pose = cloudPoses6D->back();
            pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>);
            tmp = transCloud(laserCloudFullLast, pose);
            publishCloud(pubLaserCloudSurround, tmp, timeLaserInfoCur, mapFrame);
        }
    }
    void pubMapThread()
    {
        while (1)
        {
            publishCloud(pubMapWorld, cloudGlobalMapDS, timeLaserInfoCur, mapFrame);
            std::chrono::milliseconds dura(1000);
            std::this_thread::sleep_for(dura);
        }
    }
    void printTxt()
    {
        ofstream outfile;
        outfile.open("/home/cxy/pose.txt");
        for (int i = 0; i < cloudPoses6D->size(); ++i)
        {
            myPointTypePose pose = cloudPoses6D->points[i];
            // tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw);
            outfile
                << setfill('0') << setiosflags(ios::fixed) << setprecision(9) << pose.time << " "
                << setfill('0') << setiosflags(ios::fixed) << setprecision(5) << pose.x << " " << pose.y
                << " " << pose.z << " " << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw << endl;
        }
        outfile.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ReLocalization");

    ROS_INFO("\033[1;32m----> start ReLocalization. \033[0m");

    mapOptimization mapOptimization_;
    // ros::MultiThreadedSpinner spinner(2);
    std::thread pubMapThread(&mapOptimization::pubMapThread, &mapOptimization_);
    std::thread initLocalizationThread{&mapOptimization::relocalizationThread, &mapOptimization_};
    ros::spin();
    // mapOptimization_.printTxt();
    return 0;
}
