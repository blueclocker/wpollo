/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-03 19:21:20
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 20:01:09
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/reference_path_smoother/tension_smoother_2.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by ljn on 20-5-4.
//

#ifndef TENSION_SMOOTHER_2_HPP_
#define TENSION_SMOOTHER_2_HPP_
#include <vector>
#include <cfloat>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "path_boost/reference_path_smoother/tension_smoother.hpp"

namespace PathBoostNS {
class TensionSmoother2 final : public TensionSmoother {
 public:
    TensionSmoother2() = delete;
    TensionSmoother2(const std::vector<BState> &input_points,
               const BState &start_state,
               const Map &grid_map);
    ~TensionSmoother2() override = default;

 private:
    bool osqpSmooth(const std::vector<double> &x_list,
                    const std::vector<double> &y_list,
                    const std::vector<double> &angle_list,
                    const std::vector<double> &k_list,
                    const std::vector<double> &s_list,
                    std::vector<double> *result_x_list,
                    std::vector<double> *result_y_list,
                    std::vector<double> *result_s_list) override;
    void setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const override;
    void setConstraintMatrix(const std::vector<double> &x_list,
                             const std::vector<double> &y_list,
                             const std::vector<double> &angle_list,
                             const std::vector<double> &k_list,
                             const std::vector<double> &s_list,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const override;
    void setGradient(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     Eigen::VectorXd *gradient);
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
