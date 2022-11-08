/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 19:32:10
 * @LastEditTime: 2022-11-06 21:41:26
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation_gnss.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGATION_GNSS_H_
#define NAVAGATION_GNSS_H_

#include "navagation.h"
#include <fsd_common_msgs/Comb.h>

namespace navagation
{
class NavagationGnss : public NavagationBase
{
private:
    //初始imu矫正
    int imucount_;
    Eigen::Vector3d sum_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_acc_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> rot_ = Eigen::Matrix<double, 3, 3>::Identity();
    bool imuinit_flag_;
    //矫正的imu
    Eigen::Vector3d adjusted_acc_;

    template <typename Derived> Eigen::Matrix<typename Derived::Scalar, 3, 3> GetSkewMatrix(const Eigen::MatrixBase<Derived> &v);
    template <typename Derived> Eigen::Matrix<typename Derived::Scalar, 3, 3> Amatrix(const Eigen::MatrixBase<Derived> &v);
    void ImuInit(const Eigen::Vector3d &imuMsg);

    void GpsCallback(const fsd_common_msgs::Comb::ConstPtr &msg);
public:
    NavagationGnss(ros::NodeHandle &n);
    virtual ~NavagationGnss() override = default;
    virtual void Process() override;
};


}// namespace navagation

#endif