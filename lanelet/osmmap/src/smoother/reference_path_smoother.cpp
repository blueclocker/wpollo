/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-04 15:13:14
 * @LastEditTime: 2022-11-23 15:48:24
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/smoother/reference_path_smoother.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "smoother/reference_path_smoother.h"
#include "OsqpEigen/OsqpEigen.h"
#include <chrono>

namespace plan
{
ReferencePathSmoother::ReferencePathSmoother(const std::vector<map::centerway::CenterPoint3D> &roughpoints)
{
    Spline2D csp_obj(roughpoints);
    for(double i = 0; i < csp_obj.s.back(); i += 0.2)
    {
        std::array<double, 3> point = csp_obj.calc_postion(i);
        x_list_.emplace_back(point[0]);
        y_list_.emplace_back(point[1]);
        z_list_.emplace_back(point[2]);
        s_list_.emplace_back(i);
        angle_list_.emplace_back(csp_obj.calc_yaw(i));
        k_list_.emplace_back(csp_obj.calc_curvature(i));
        if(i >= 30) break;
    }
    // std::cout << "reference path smoother init success" << std::endl;
}

ReferencePathSmoother::~ReferencePathSmoother()
{
}

bool ReferencePathSmoother::Smooth()
{
    const auto start_timestamp = std::chrono::system_clock::now();
    std::vector<double> result_x_list, result_y_list, result_s_list;
    bool solver_ok = OsqpSmooth(&result_x_list,
                                &result_y_list,
                                &result_s_list);
    if (!solver_ok) 
    {
        std::cout << "Tension smoother failed!" << std::endl;
        return false;
    }

    x_list_ = std::move(result_x_list);
    y_list_ = std::move(result_y_list);
    s_list_ = std::move(result_s_list);

    const auto now_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now_timestamp - start_timestamp;
    std::cout << "osqp total time is " << diff.count() * 1000.0 << " ms." << std::endl;
    return true;
}

bool ReferencePathSmoother::OsqpSmooth(std::vector<double> *result_x_list,
                                       std::vector<double> *result_y_list,
                                       std::vector<double> *result_s_list) const
{
    CHECK_EQ(x_list_.size(), y_list_.size());
    CHECK_EQ(y_list_.size(), angle_list_.size());
    CHECK_EQ(angle_list_.size(), s_list_.size());
    auto point_num = x_list_.size();
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(3 * point_num);
    solver.data()->setNumberOfConstraints(3 * point_num);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    SetHessianMatrix(point_num, &hessian);
    SetConstraintMatrix(&linearMatrix, &lowerBound, &upperBound);
    // Input to solver.
    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver.initSolver()) return false;
    if (static_cast<int>(solver.solveProblem()) != 0) return false;
    const auto &QPSolution{solver.getSolution()};
    // Output.
    result_s_list->clear();
    result_x_list->clear();
    result_y_list->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != point_num; ++i) 
    {
        double tmp_x = QPSolution(i);
        double tmp_y = QPSolution(point_num + i);
        result_x_list->emplace_back(tmp_x);
        result_y_list->emplace_back(tmp_y);
        if (i != 0)
            tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                              + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}

void ReferencePathSmoother::SetHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const 
{
    const size_t x_start_index{0};
    const size_t y_start_index{x_start_index + size};
    const size_t d_start_index{y_start_index + size};
    const size_t matrix_size = 3 * size;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // Curvature part.
    Eigen::Matrix<double, 3, 1> dds_vec{1, -2, 1};
    Eigen::Matrix3d dds_part{dds_vec * dds_vec.transpose() * 1}; //FLAGS_cartesian_curvature_weight
    Eigen::Matrix<double, 4, 1> ddds_vec{-1, 3, -3, 1};
    Eigen::Matrix4d ddds_part{ddds_vec * ddds_vec.transpose() * 50}; //FLAGS_cartesian_curvature_rate_weight
    for (int i = 0; i != size - 2; ++i) 
    {
        hessian.block(x_start_index + i, x_start_index + i, 3, 3) += dds_part;
        hessian.block(y_start_index + i, y_start_index + i, 3, 3) += dds_part;
        if (i != size - 3) 
        {
            hessian.block(x_start_index + i, x_start_index + i, 4, 4) += ddds_part;
            hessian.block(y_start_index + i, y_start_index + i, 4, 4) += ddds_part;
        }
    }
    // Deviation part.
    for (int i = 0; i != size; ++i) 
    {
        hessian(d_start_index + i, d_start_index + i) = 0; //FLAGS_cartesian_deviation_weight
    }
    *matrix_h = hessian.sparseView();
}

void ReferencePathSmoother::SetConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                                Eigen::VectorXd *lower_bound,
                                                Eigen::VectorXd *upper_bound) const 
{
    const size_t size{x_list_.size()};
    const size_t x_start_index{0};
    const size_t y_start_index{x_start_index + size};
    const size_t d_start_index{y_start_index + size};
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * size, 3 * size);
    *lower_bound = Eigen::MatrixXd::Zero(3 * size, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * size, 1);
    for (int i = 0; i != size; ++i) 
    {
        // x, y and d
        cons(x_start_index + i, x_start_index + i) = cons(y_start_index + i, y_start_index + i) = 1;
        double theta{angle_list_[i] + M_PI_2};
        cons(x_start_index + i, d_start_index + i) = -cos(theta);
        cons(y_start_index + i, d_start_index + i) = -sin(theta);
        // d
        cons(d_start_index + i, d_start_index + i) = 1;
        // bounds
        (*lower_bound)(x_start_index + i) = x_list_[i];
        (*upper_bound)(x_start_index + i) = x_list_[i];
        (*lower_bound)(y_start_index + i) = y_list_[i];
        (*upper_bound)(y_start_index + i) = y_list_[i];
    }
    *matrix_constraints = cons.sparseView();
    // d bounds.
    (*lower_bound)(d_start_index) = 0;
    (*upper_bound)(d_start_index) = 0;
    (*lower_bound)(d_start_index + size - 1) = -0.5;
    (*upper_bound)(d_start_index + size - 1) = 0.5;
    const double default_clearance = 2;
    // const double shrink_clearance = 0;
    for (size_t i = 1; i != size - 1; ++i) 
    {
        double x = x_list_[i];
        double y = y_list_[i];
        double clearance = 1.5;
        // Adjust clearance.
        clearance = std::min(clearance, default_clearance);
        //    isEqual(clearance, 0) ? default_clearance :
        //           clearance > shrink_clearance ? clearance - shrink_clearance : clearance;
        (*lower_bound)(d_start_index + i) = -clearance;
        (*upper_bound)(d_start_index + i) = clearance;
    }
}


} //namespace plan
