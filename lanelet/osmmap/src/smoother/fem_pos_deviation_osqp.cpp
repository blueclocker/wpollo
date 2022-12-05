/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-23 13:25:13
 * @LastEditTime: 2022-11-23 18:43:02
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/smoother/fem_pos_deviation_osqp.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "smoother/fem_pos_deviation_osqp.h"
#include "tools/cubic_path.h"
#include <glog/logging.h>
#include <OsqpEigen/OsqpEigen.h>

namespace plan
{
FemPosDeviationOsqp::FemPosDeviationOsqp(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z)
{
    CHECK_EQ(x.size(), y.size());
    CHECK_EQ(y.size(), z.size());
    math::CubicPath frenetpath(x, y, z);
    for(double s = 0.0; s < frenetpath.GetLength(); s += 1.0)
    {
        std::array<double, 3> frenetpoint = frenetpath.GetPosition(s);
        x_raw_.emplace_back(frenetpoint[0]);
        y_raw_.emplace_back(frenetpoint[1]);
        z_raw_.emplace_back(frenetpoint[2]);
        if(s >= 80) break;
    }
    num_ = x_raw_.size();
}

void FemPosDeviationOsqp::SetBounds(const double bound, Eigen::VectorXd *lowerBound_h, Eigen::VectorXd *upperBound_h) const
{
    Eigen::VectorXd lowbound = Eigen::VectorXd::Zero(2 * num_);
    Eigen::VectorXd upbound = Eigen::VectorXd::Zero(2 * num_);
    for(int i = 0; i < num_; ++i)
    {
        //xraw[i]-bound <= x[i] <= xraw[i]+bound
        lowbound(2 * i) = x_raw_[i] - bound;
        upbound(2 * i) = x_raw_[i] + bound;
        //yraw[i]-bound <= y[i] <= yraw[i]+bound
        lowbound(2 * i + 1) = y_raw_[i] - bound;
        upbound(2 * i + 1) = y_raw_[i] + bound;
    }
    *lowerBound_h = lowbound;
    *upperBound_h = upbound;
}

void FemPosDeviationOsqp::SetHessianMatrix(Eigen::SparseMatrix<double> *matrix_h, Eigen::VectorXd *gradient_h) const
{
    int num_variables = 2 * num_;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(num_variables, num_variables);
    Eigen::Matrix<double, 6, 2> A_1;
    A_1 << 1, 0, 0, 1, -2, 0, 0, -2, 1, 0, 0, 1;
    Eigen::Matrix<double, 6, 6> A = A_1 * A_1.transpose() * weight_ref_[0];
    Eigen::Matrix<double, 4, 2> B_1;
    B_1 << 1, 0, 0, 1, -1, 0, 0, -1;
    Eigen::Matrix<double, 4, 4> B = B_1 * B_1.transpose() * weight_ref_[1];
    for(int i = 0; i < num_; ++i)
    {
        hessian(2 * i, 2 * i) += weight_ref_[2];
        hessian(2 * i + 1, 2 * i + 1) += weight_ref_[2];
        if(i < num_ - 2)
        {
            hessian.block<6, 6>(i, i) += A;
        }
        if(i < num_ - 1)
        {
            hessian.block<4, 4>(i, i) += B;
        }
    }
    *matrix_h = hessian.sparseView();

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(2 * num_);
    for(int i = 0; i < num_; ++i)
    {
        gradient(2 * i) = x_raw_[i];
        gradient(2 * i + 1) = y_raw_[i];
    }
    *gradient_h = gradient;
}

void FemPosDeviationOsqp::SetConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints) const
{
    Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(num_, num_);
}

bool FemPosDeviationOsqp::Solve(std::vector<double> *result_x_list, std::vector<double> *result_y_list, 
                                std::vector<double> *result_z_list) const
{
    if(num_ <= 2) 
    {
        std::cout << "point number < 3" << std::endl;
        return false;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(2 * num_);
    solver.data()->setNumberOfConstraints(2 * num_);

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    SetHessianMatrix(&hessian, &gradient);

    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    SetBounds(0.2, &lowerBound, &upperBound);
    Eigen::MatrixXd constraints = Eigen::MatrixXd::Identity(2 * num_, 2 * num_);
    Eigen::SparseMatrix<double> linearMatrix;
    linearMatrix = constraints.sparseView();

    if(!solver.data()->setHessianMatrix(hessian)) return false;
    if(!solver.data()->setGradient(gradient)) return false;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if(!solver.data()->setLowerBound(lowerBound)) return false;
    if(!solver.data()->setUpperBound(upperBound)) return false;
    //solve
    if(!solver.initSolver()) return false;
    if(static_cast<int>(solver.solveProblem()) != 0) return false;
    auto qpsolution = solver.getSolution();
    
    for(int i = 0; i < num_; ++i)
    {
        result_x_list->emplace_back(qpsolution(2 * i));
        result_y_list->emplace_back(qpsolution(2 * i + 1));
    }
    *result_z_list = z_raw_;
    return true;
}



}//namespace plan
