/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:36
 * @LastEditTime: 2022-04-23 19:58:47
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_io.h
 */
#ifndef MAP_CORE_H_
#define MAP_CORE_H_

#include "map_io.h"
#include "visualization.h"
#include "map_plan.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <fsd_common_msgs/comb.h>
#include <nav_msgs/Path.h>
#include <osmmap/navigation.h>
#include <osmmap/regulatoryelement.h>
#include <osmmap/carState.h>
#include <osmmap/Lane.h>
#include <osmmap/Lanes.h>

/*地图读取/可视化模块
* 调用node, way, relation, centerway四大类实现地图解析
* 调用visualization实现可视化
*/

namespace map
{
class HDMap
{
private:
    const node::Node *nodesptr;
    const way::Way *waysptr;
    const relation::Relation *relationsptr;
    const centerway::CenterWay *centerwaysptr;
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
    ros::Publisher gpspath_pub;
    ros::Publisher carstate_pub;
    ros::Publisher navigation_pub;
    ros::Publisher lanes_pub;

    visualization_msgs::MarkerArray map_markerarray;
    visualization_msgs::MarkerArray path_markerarray;
    nav_msgs::Path gpspath;
    osmmap::navigation laneletinfo;
    osmmap::Lanes Lanesinfo;

    tf::TransformBroadcaster broadcaster;
    tf::Transform baselink2map;
    void fullNavigationInfo();
    void fullLanesInfo(const int id_);
public:
    HDMap(ros::NodeHandle n_);
    ~HDMap();
    void Smoothpath(const std::vector<int> &pathid_);
    void startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void goalpoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void gps_callback(const fsd_common_msgs::comb::ConstPtr &msg);
};

};//namespace map

#endif