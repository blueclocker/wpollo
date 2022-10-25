/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-04 15:14:11
 * @LastEditTime: 2022-10-04 16:59:45
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/smoother/reference_path_smoother.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef REFERENCE_PATH_SMOOTHER_H_
#define REFERENCE_PATH_SMOOTHER_H_

#include "osmmap/centerway.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "osmmap/cubic_spline.h"
#include "glog/logging.h"
#include "gflags/gflags.h"

namespace plan
{
class ReferencePathSmoother
{
private:
    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> z_list_;
    std::vector<double> s_list_;
    std::vector<double> angle_list_;
    std::vector<double> k_list_;

    bool OsqpSmooth(std::vector<double> *result_x_list, std::vector<double> *result_y_list, std::vector<double> *result_s_list) const;
    void SetHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const;
    void SetConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints, Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound) const;
public:
    ReferencePathSmoother() = delete;
    explicit ReferencePathSmoother(const std::vector<map::centerway::CenterPoint3D> &roughpoints);
    bool Smooth();
    std::vector<double> GetXList() const {return x_list_;}
    std::vector<double> GetYList() const {return y_list_;}
    std::vector<double> GetZList() const {return z_list_;}
    std::vector<double> GetSList() const {return s_list_;}
    std::vector<double> GetAngleList() const {return angle_list_;}
    std::vector<double> GetKList() const {return k_list_;}
    ~ReferencePathSmoother();
};







} //namespace plan

#endif