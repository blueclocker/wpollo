/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-05-06 09:55:33
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-05-31 13:55:09
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/dubins.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef DUBINS_H
#define DUBINS_H

#include <boost/math/constants/constants.hpp>
#include <cassert>

class DubinsStateSpace {
public:
    /** \brief The Dubins path segment type */
    enum DubinsPathSegmentType {
        DUBINS_LEFT = 0, DUBINS_STRAIGHT = 1, DUBINS_RIGHT = 2
    };
    /** \brief Dubins path types */
    static const DubinsPathSegmentType dubinsPathType[6][3];

    /** \brief Complete description of a Dubins path */
    class DubinsPath {
    public:
        DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
                   double p = std::numeric_limits<double>::max(), double q = 0.) : type_(type) {
            length_[0] = t;
            length_[1] = p;
            length_[2] = q;
            assert(t >= 0.);
            assert(p >= 0.);
            assert(q >= 0.);
        }

        double length() const {
            return length_[0] + length_[1] + length_[2];
        }

        /** Path segment types */
        const DubinsPathSegmentType *type_;
        /** Path segment lengths */
        double length_[3];
    };

    DubinsStateSpace(double turningRadius = 1.0) : rho_(turningRadius) {}

    void sample(double q0[3], double q1[3], double step_size, double &length, std::vector<std::vector<double> > &points);

    double distance(double q0[3], double q1[3]);


    /** \brief Return the shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
    DubinsPath dubins(double q0[3], double q1[3]);

protected:

    /** \brief Turning radius */
    double rho_;

    void interpolate(double q0[3], DubinsPath &path, double seg, double s[3]);

};


#endif //R_S_PLANNER_DUBINS_H
