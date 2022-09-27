/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-06 21:48:33
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-12 19:39:40
 * @FilePath: /wpollo/src/lanelet/path_boost/include/hastar/type.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TYPE_HPP_
#define TYPE_HPP_

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/StdVector>
#include "backward.hpp"

namespace HybidA
{
// template<int dim>
// using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
//         Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

// typedef TypeVectorVecd<4> VectorVec4d;
// typedef TypeVectorVecd<3> VectorVec3d;
// typedef TypeVectorVecd<2> VectorVec2d;

typedef typename std::vector<Eigen::Matrix<double, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1> > > VectorVec4d;
typedef typename std::vector<Eigen::Matrix<double, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1> > > VectorVec3d;
typedef typename std::vector<Eigen::Matrix<double, 2, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 1> > > VectorVec2d;

typedef typename Eigen::Vector2d Vec2d;
typedef typename Eigen::Vector3d Vec3d;
typedef typename Eigen::Vector4d Vec4d;

typedef typename Eigen::Vector2i Vec2i;
typedef typename Eigen::Vector3i Vec3i;

typedef typename Eigen::Matrix2d Mat2d;
typedef typename Eigen::Matrix3d Mat3d;

typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;



}
#endif