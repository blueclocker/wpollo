/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:36
 * @LastEditTime: 2022-09-25 15:16:50
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
T constrainAngle(T angle)
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
    const node::Node *nodesptr;
    const way::Way *waysptr;
    const relation::Relation *relationsptr;
    const centerway::CenterWay *centerwaysptr;
    const grid_map::GridMap *gridmapsptr;
    map::Map *vectormap;
    MapVisualization *visualmap;
    plan::Globalplan *globalplans;

    //params
    //起点所在路段id
    int start_path;
    //终点所在路段id
    int end_path;
    //起点的前一个中心线点id
    int start_centerpoint_id;
    //终点的下一个中心线点id
    int end_centerpoint_id;
    //是否存在有效起点
    bool isstart_path_exist;
    //是否存在有效终点
    bool isend_path_exist;
    //起点状态, x,y,yaw
    double start_state[3];
    //终点状态
    double end_state[3];
    geometry_msgs::Pose endPose;
    //初始imu矫正
    int imucount;
    Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_acc = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> rot = Eigen::Matrix<double, 3, 3>::Identity();
    bool imuinit_flag;
    //矫正的imu
    Eigen::Vector3d adjusted_acc;
    //规划结果，得到的道路中心线id
    std::vector<int> paths;
    node::Point3D *atnowpoint;//当前点相对坐标
    std::vector<centerway::CenterPoint3D> smoothpathnode;

    //ros
    ros::NodeHandle n;
    ros::Time currenttime;
    ros::Subscriber startpoint_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber goalpoint_sub;

    ros::Publisher map_pub;
    ros::Publisher path_pub;
    ros::Publisher gridmap_pub;
    ros::Publisher gpspath_pub;
    ros::Publisher carstate_pub;
    ros::Publisher navigation_pub;
    ros::Publisher lanes_pub;
    ros::Publisher golbalpath_pub;

    visualization_msgs::MarkerArray map_markerarray;
    visualization_msgs::MarkerArray path_markerarray;
    nav_msgs::OccupancyGrid gridmapmsg;
    nav_msgs::Path gpspath;
    osmmap::Navigation laneletinfo;
    osmmap::Lanes Lanesinfo;

    tf::TransformBroadcaster broadcaster;
    tf::Transform baselink2map;
    void fullNavigationInfo();
    void fullLanesInfo(const int id_);
    template <typename Derived> Eigen::Matrix<typename Derived::Scalar, 3, 3> GetSkewMatrix(const Eigen::MatrixBase<Derived> &v);
    template <typename Derived> Eigen::Matrix<typename Derived::Scalar, 3, 3> Amatrix(const Eigen::MatrixBase<Derived> &v);
    void imuInit(const Eigen::Vector3d &imuMsg);
    void pushCenterPoint(const std::vector<int> &pathid_);
    void outMapPlan(const centerway::CenterPoint3D &atnow_centerpoint, const double heading);

public:
    HDMap(ros::NodeHandle n_);
    ~HDMap();
    void Smoothpath();
    void startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void goalpoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void gps_callback(const fsd_common_msgs::Comb::ConstPtr &msg);
};

};//namespace map

#endif