/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-23 13:25:34
 * @LastEditTime: 2022-11-23 19:15:30
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/smoother/fem_pos_deviation_osqp.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef FEM_POS_DEVIATION_OSQP_H_
#define FEM_POS_DEVIATION_OSQP_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace plan
{
class FemPosDeviationOsqp
{
private:
    std::vector<double> x_raw_;//frenet->s
    std::vector<double> y_raw_;//frenet->l
    std::vector<double> z_raw_;//ele
    // std::vector<double> lower_bounds_;
    // std::vector<double> upper_bounds_;
    int num_ = 0;
    std::array<double, 3> weight_ref_ = {0.0, 0.0, 0.0};//权重系数, w_smooth, w_length, w_ref

    // bool OsqpSmooth(std::vector<double> *result_x_list, std::vector<double> *result_y_list) const;
    void SetHessianMatrix(Eigen::SparseMatrix<double> *matrix_h, Eigen::VectorXd *gradient_h) const;
    void SetConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints) const;
public:
    FemPosDeviationOsqp(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z);
    void SetWeightRef(const std::array<double, 3> &w) {weight_ref_ = w;}
    void SetBounds(const double bound, Eigen::VectorXd *lowerBound_h, Eigen::VectorXd *upperBound_h) const;
    bool Solve(std::vector<double> *result_x_list, std::vector<double> *result_y_list, std::vector<double> *result_z_list) const;
    ~FemPosDeviationOsqp() = default;
};



}//namespace plan

#endif