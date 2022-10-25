/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-08-28 19:44:33
 * @LastEditors: error: git config user.name && git config user.email & please set dead value or install git
 * @LastEditTime: 2022-09-20 15:47:47
 */
#pragma once
#include <string>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;
#define POS_ 0
#define ROT_ 3
#define VEL_ 6
#define BIA_ 9
#define BIG_ 12
#define GW_ 15
//终端字体颜色
#define RESET "\033[0m"
#define BLACK "\033[30m"     /* Black */
#define RED "\033[1;31m"     /* Red */
#define GREEN "\033[1;32m"   /* Green */
#define YELLOW "\033[1;33m"  /* Yellow */
#define BLUE "\033[1;34m"    /* Blue */
#define MAGENTA "\033[1;35m" /* Magenta */
#define CYAN "\033[1;36m"    /* Cyan */
#define WHITE "\033[1;37m"   /* White */
enum LidarType
{
    VELODYNE,
    RSLIDAR
};

extern const string imuFrame = "base_link";
extern const string lidarFrame = "velodyne";
extern const string odomFrame = "odom";
extern const string mapFrame = "map";
extern const string pointCloudTopic = "/velodyne_points";
extern const LidarType lidarType = LidarType::VELODYNE;
extern const string imuTopic = "/imu_repub";
// LIDAR->IMU
extern const Vector3f extTrans = {0, 0, -0.05};
// extern const Vector3f extTrans = {0.27255, -0.00053, 0.17954};
extern const Quaternionf extQuat = {1, 0, 0, 0};
// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float scanPeriod = 0.1;
extern const float ang_res_x = 0.2;
extern const float minSensorRange = 1.0;
extern const float maxSensorRange = 200.0;
extern const float delta_radius_ = 0.2;

// Segment
extern const int kLidarRows0 = 16;
extern const int kLidarCols0 = 1800;
extern const int kNumSectors0 = 360;
extern const float kLidarHorizRes0 = 0.2;
extern const float kLidarVertRes0 = 2.0;
extern const float kLidarVertFovMax0 = 15;
extern const float kLidarVertFovMin0 = -15;
// extern const float kLidarVertFovMax0 = 15;
// extern const float kLidarVertFovMin0 = -55;
extern const float kLidarProjectionError0 = 0.5;
extern const std::vector<float> kExtrinsicTrans0 = {0.0, 0.0, 1.0}; // extern const std::vector<float> kExtrinsicTrans0 = {0.0, 0.0, 2.0};
extern const std::vector<float> kExtrinsicRot0 = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
extern const float kGroundSameLineTolerance0 = 2; // degree(0.035, around 0.1m / 3m)
extern const float kGroundSlopeTolerance0 = 10;   // 10 degrees(0.176)
extern const float kGroundYInterceptTolerance0 = 0.5;
extern const float kGroundPointLineDistThres0 = 0.1;

// extern const float kWallSameLineTolerance = 10; // 10 degrees(0.176), 6 degree(0.1051), 3 degree(0.0524)
// extern const float kWallSlopeTolerance = 75;    // 85 degrees(11.43), 80 degrees(5.67), 75 degrees(3.73)
// extern const float kWallLineMinBinNum = 3;      // a wall line should cover at least 3 rings.
// extern const float kWallPointLineDistThres = 0.1;

// IMU
extern const float Gravity = 9.80544;
extern const float imuAccNoise = 0.1;
extern const float imuGyrNoise = 0.01;
extern const float imuAccBiasN = 0.0001;
extern const float imuGyrBiasN = 0.0001;
extern const float deg = M_PI / 180.0;        // degree
extern const float rad = 180.0 / M_PI;        // radian
extern const float dph = deg / 3600.0;        // degree per hour
extern const float dpsh = deg / sqrt(3600.0); // degree per square-root hour
extern const float mg = Gravity / 1000.0;     // mili-gravity force
extern const float ug = mg / 1000.0;          // micro-gravity force
extern const float mgpsHz = mg / sqrt(1.0);   // mili-gravity force per second
extern const float ugpsHz = ug / sqrt(1.0);   // micro-gravity force per second
extern const float ACC_N = 70000;
extern const float ACC_W = 500;
extern const float GYR_N = 0.1;
extern const float GYR_W = 0.05;
// Odometry
extern const float LIDAR_STD = 0.001;
extern const float leafSize = 0.2;
extern const float SEARCH_DIST_THRESHOLD = 1.0;
extern const float ESTI_PLANE_THRESHOLD = 0.1;
extern const int NUM_MATCH_POINTS = 5;
extern const int MIN_NUM_MATCH_POINTS = 3;
extern const float optimizationStep = 1.0;

// mapping
extern const int surroundingKeyframeSearchNum = 50;
extern const float surroundingKeyframeSearchRadius = 80.0;
extern const float historyKeyframeSearchRadius = 40.0;
extern const int historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.8;
extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;
extern const int loopClosureFrequency = 1;
extern const float historyKeyframeSearchTimeDiff = 30.0;

// ieskf
extern const float ivox_grid_resolution = 0.5;
extern const int ivox_nearby_type = 26;
extern const int ivox_capacity = 1000000;
extern const int cube_side_length = 1000;
extern const int point_filter_num = 1;
extern const float filter_size_map = 0.5;
extern const float esti_plane_threshold = 0.1;