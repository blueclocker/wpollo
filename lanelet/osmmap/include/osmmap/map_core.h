/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:36
 * @LastEditTime: 2022-10-03 19:20:15
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_core.h
 */
#ifndef MAP_CORE_H_
#define MAP_CORE_H_

#include "map_io.h"
#include "visualization.h"
#include "map_plan.h"
#include "dubins.h"
#include "cubic_spline.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <fsd_common_msgs/Comb.h>
#include <nav_msgs/Path.h>
#include <osmmap/Navigation.h>
#include <osmmap/Regulatoryelement.h>
#include <osmmap/CarState.h>
#include <osmmap/Lane.h>
#include <osmmap/Lanes.h>

/*地图读取/可视化模块
* 调用node, way, relation, centerway四大类实现地图解析
* 调用visualization实现可视化
*/

namespace map
{
template <typename T>
T ConstrainAngle(T angle)
{
    if(angle < 180 && angle > 90)
    {
        angle -= 270;
    }else{
        angle += 90;
    }
    return angle;
}

class HDMap
{
private:
    const node::Node *nodesptr_;
    const way::Way *waysptr_;
    const relation::Relation *relationsptr_;
    const centerway::CenterWay *centerwaysptr_;
    const grid_map::GridMap *gridmapsptr_;
    map::Map *vectormap_;
    MapVisualization *visualmap_;
    plan::Globalplan *globalplans_;

    //params
    //起点所在路段id
    int start_path_;
    //终点所在路段id
    int end_path_;
    //起点的前一个中心线点id
    int start_centerpoint_id_;
    //终点的下一个中心线点id
    int end_centerpoint_id_;
    //是否存在有效起点
    bool isstart_path_exist_;
    //是否存在有效终点
    bool isend_path_exist_;
    //起点状态, x,y,yaw
    double start_state_[3];
    //终点状态
    double end_state_[3];
    geometry_msgs::Pose endPose_;
    //初始imu矫正
    int imucount_;
    Eigen::Vector3d sum_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_acc_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> rot_ = Eigen::Matrix<double, 3, 3>::Identity();
    bool imuinit_flag_;
    //矫正的imu
    Eigen::Vector3d adjusted_acc_;
    //规划结果，得到的道路中心线id
    std::vector<int> paths_;
    node::Point3D *atnowpoint_;//当前点相对坐标
    std::vector<centerway::CenterPoint3D> smoothpathnode_;

    //ros
    ros::NodeHandle n_;
    ros::Time currenttime_;
    ros::Subscriber startpoint_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber goalpoint_sub_;

    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher gridmap_pub_;
    ros::Publisher gpspath_pub_;
    ros::Publisher carstate_pub_;
    ros::Publisher navigation_pub_;
    ros::Publisher lanes_pub_;
    ros::Publisher golbalpath_pub_;

    visualization_msgs::MarkerArray map_markerarray_;
    visualization_msgs::MarkerArray path_markerarray_;
    nav_msgs::OccupancyGrid gridmapmsg_;
    nav_msgs::Path gpspath_;
    osmmap::Navigation laneletinfo_;
    osmmap::Lanes lanesinfo_;

    tf::TransformBroadcaster broadcaster_;
    tf::Transform baselink2map_;
    void FullNavigationInfo();
    void FullLanesInfo(const int id);
    template <typename Derived> Eigen::Matrix<typename Derived::Scalar, 3, 3> GetSkewMatrix(const Eigen::MatrixBase<Derived> &v);
    template <typename Derived> Eigen::Matrix<typename Derived::Scalar, 3, 3> Amatrix(const Eigen::MatrixBase<Derived> &v);
    void ImuInit(const Eigen::Vector3d &imuMsg);
    void PushCenterPoint(const std::vector<int> &pathid);
    void OutMapPlan(const centerway::CenterPoint3D &atnow_centerpoint, const double heading);

public:
    HDMap(ros::NodeHandle n);
    ~HDMap();
    void SmoothPath();
    void StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void GpsCallback(const fsd_common_msgs::Comb::ConstPtr &msg);
};

};//namespace map

#endif