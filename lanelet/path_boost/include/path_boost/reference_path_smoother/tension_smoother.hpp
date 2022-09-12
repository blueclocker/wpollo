/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-03 19:21:20
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 20:02:12
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/reference_path_smoother/tension_smoother.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef TENSION_SMOOTHER_HPP_
#define TENSION_SMOOTHER_HPP_
#include <vector>
#include <cfloat>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "path_boost/reference_path_smoother/reference_path_smoother.hpp"

namespace PathBoostNS {
class TensionSmoother : public ReferencePathSmoother {
 public:
    TensionSmoother() = delete;
    TensionSmoother(const std::vector<BState> &input_points,
                    const BState &start_state,
                    const Map &grid_map);
    ~TensionSmoother() override = default;

 private:
    bool smooth(std::shared_ptr<PathBoostNS::ReferencePath> reference_path) override;
    virtual bool osqpSmooth(const std::vector<double> &x_list,
                            const std::vector<double> &y_list,
                            const std::vector<double> &angle_list,
                            const std::vector<double> &k_list,
                            const std::vector<double> &s_list,
                            std::vector<double> *result_x_list,
                            std::vector<double> *result_y_list,
                            std::vector<double> *result_s_list);
    virtual void setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const;
    virtual void setConstraintMatrix(const std::vector<double> &x_list,
                                     const std::vector<double> &y_list,
                                     const std::vector<double> &angle_list,
                                     const std::vector<double> &k_list,
                                     const std::vector<double> &s_list,
                                     Eigen::SparseMatrix<double> *matrix_constraints,
                                     Eigen::VectorXd *lower_bound,
                                     Eigen::VectorXd *upper_bound) const;
};

}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_
