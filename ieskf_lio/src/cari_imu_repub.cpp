/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-09-15 19:34:27
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-09-16 14:57:32
 */
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <Eigen/Core>
// #include <math.h>
// #include <cmath>
// #include <iostream>
// #include <fstream>

// using namespace std;
// using namespace Eigen;

// ros::Publisher imu_pub;

// void combCallback(const sensor_msgs::ImuPtr &msg)
// {
//     sensor_msgs::Imu imuMsg = *msg;
//     // imuMsg.linear_acceleration.x = msg->linear_acceleration.y + 0.132932;
//     imuMsg.linear_acceleration.x = msg->linear_acceleration.y;
//     // imuMsg.linear_acceleration.y = -(msg->linear_acceleration.x + 0.0271338);
//     imuMsg.linear_acceleration.y = -msg->linear_acceleration.x;
//     imuMsg.linear_acceleration.z = msg->linear_acceleration.z;
//     imuMsg.angular_velocity.x = msg->angular_velocity.y;
//     imuMsg.angular_velocity.y = -msg->angular_velocity.x;
//     imuMsg.angular_velocity.z = msg->angular_velocity.z;

//     double sum = sqrt(imuMsg.linear_acceleration.x * imuMsg.linear_acceleration.x +
//                       imuMsg.linear_acceleration.y * imuMsg.linear_acceleration.y +
//                       imuMsg.linear_acceleration.z * imuMsg.linear_acceleration.z);

//     std::cout << "SUM:  " << sum << std::endl;

//     imu_pub.publish(imuMsg);
// }

// int main(int argc, char **argv)
// {

//     ros::init(argc, argv, "imuRepub");
//     ROS_INFO("\033[1;32m----> start imuRepub. \033[0m");
//     ros::NodeHandle n;
//     ros::Subscriber sub = n.subscribe("imu", 1, combCallback);
//     imu_pub = n.advertise<sensor_msgs::Imu>("/imu_repub", 5);
//     ros::spin();
//     return 0;
// }
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "../include/ieskf_lio/common.h"

using namespace std;
using namespace Eigen;

ros::Publisher imu_pub;

void combCallback(const sensor_msgs::ImuPtr &msg)
{
    sensor_msgs::Imu imuMsg = *msg;
    imuMsg.linear_acceleration.x = msg->linear_acceleration.y;
    imuMsg.linear_acceleration.y = -msg->linear_acceleration.x;
    imuMsg.linear_acceleration.z = msg->linear_acceleration.z;
    imuMsg.angular_velocity.x = msg->angular_velocity.y;
    imuMsg.angular_velocity.y = -msg->angular_velocity.x;
    imuMsg.angular_velocity.z = msg->angular_velocity.z;

    imu_pub.publish(imuMsg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "imuRepub");
    ROS_INFO("\033[1;32m----> start imuRepub. \033[0m");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("imu", 1, combCallback);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu_repub", 5);
    ros::spin();
    return 0;
}

// void MySigintHandler(int sig)
// {
//     //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
//     ROS_INFO("imu_repub shutting down!");
//     ros::shutdown();
// }
// class imuRepub
// {
// private:
//     ros::NodeHandle n;

//     ros::Subscriber sub;

//     ros::Publisher imu_pub;
//     ros::Publisher imu_calibration_pub;

//     int seq = 0;
//     Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
//     Eigen::Vector3d avg_acc = Eigen::Vector3d::Zero();
//     Eigen::Matrix<double, 3, 3> rot = Eigen::Matrix<double, 3, 3>::Identity();

// public:
//     imuRepub() : n("~")
//     {
//         sub = n.subscribe("/imu", 1, &imuRepub::combCallback, this);
//         imu_pub = n.advertise<sensor_msgs::Imu>("/imu_repub", 5);
//     }

//     void combCallback(const sensor_msgs::ImuPtr &msg)
//     {
//         ++seq;
//         sensor_msgs::Imu imuMsg = *msg;
//         // imuMsg.linear_acceleration.x = msg->linear_acceleration.y + 0.132932;
//         imuMsg.linear_acceleration.x = msg->linear_acceleration.y;
//         // imuMsg.linear_acceleration.y = -(msg->linear_acceleration.x + 0.0271338);
//         imuMsg.linear_acceleration.y = -msg->linear_acceleration.x;
//         imuMsg.linear_acceleration.z = msg->linear_acceleration.z;
//         imuMsg.angular_velocity.x = msg->angular_velocity.y;
//         imuMsg.angular_velocity.y = -msg->angular_velocity.x;
//         imuMsg.angular_velocity.z = msg->angular_velocity.z;

//         if (seq < 201)
//         {
//             Eigen::Vector3d acc{imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z};
//             sum_acc += acc;
//         }
//         else
//         {
//             if (seq == 201)
//             {
//                 avg_acc = sum_acc / 200.0;
//                 std::cout << "avg acc:  " << avg_acc.transpose().x() << "  " << avg_acc.transpose().y()
//                           << "  " << avg_acc.transpose().z() << std::endl;
//                 double sqrt_acc_acc = sqrt(avg_acc.x() * avg_acc.x() + avg_acc.y() * avg_acc.y() + avg_acc.z() * avg_acc.z());
//                 std::cout << "sqrt acc*acc: " << sqrt_acc_acc << std::endl;
//                 double norm = avg_acc.norm();
//                 avg_acc /= norm;
//                 Eigen::Vector3d gravity_acc(Eigen::Vector3d(0, 0, 1));
//                 double cos_value = avg_acc.dot(gravity_acc);
//                 double angle = acos(cos_value);
//                 std::cout << std::endl
//                           << std::endl
//                           << std::endl;
//                 std::cout << "The angle between Gravity and IMU-Zaxis is " << angle * 180.0 / M_PI << " degree." << std::endl;
//                 std::cout << std::endl
//                           << std::endl
//                           << std::endl;
//                 Eigen::Vector3d axis = GetSkewMatrix(avg_acc) * gravity_acc;
//                 Eigen::AngleAxisd rot_vec(angle, axis.normalized());
//                 rot = rot_vec.toRotationMatrix();
//                 Quaterniond rotQuat(rot);
//                 rotQuat.normalize();
//                 std::cout
//                     << "The Quaternion between Gravity and IMU-Zaxis is " << rotQuat.w() << " " << rotQuat.x() << " " << rotQuat.y() << " " << rotQuat.z() << std::endl;
//                 std::cout << std::endl
//                           << std::endl
//                           << std::endl;
//                 return;
//             }
//             Eigen::Vector3d acc{imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z};
//             Eigen::Vector3d adjusted_acc = rot * acc;
//             Eigen::Vector3d gyr{imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z};
//             Eigen::Vector3d adjusted_gyr = rot * gyr;
//             imuMsg.linear_acceleration.x = adjusted_acc.x();
//             imuMsg.linear_acceleration.y = adjusted_acc.y();
//             imuMsg.linear_acceleration.z = adjusted_acc.z();
//             imuMsg.angular_velocity.x = adjusted_gyr.x();
//             imuMsg.angular_velocity.y = adjusted_gyr.y();
//             imuMsg.angular_velocity.z = adjusted_gyr.z();
//             imu_pub.publish(imuMsg);
//         }
//     }
// };

// int main(int argc, char **argv)
// {

//     ros::init(argc, argv, "imuRepub");
//     ROS_INFO("\033[1;32m----> start imuRepub. \033[0m");
//     imuRepub imu_repub;
//     signal(SIGINT, MySigintHandler);
//     ros::spin();

//     return 0;
// }