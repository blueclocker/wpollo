/*
 * @Author: your name
 * @Date: 2022-03-05 17:50:28
 * @LastEditTime: 2022-10-03 15:43:30
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/visualization.h
 */
#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include "map_node.h"
#include "map_way.h"
#include "map_relation.h"
#include "centerway.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Polygon.h>

/*可视化模块，把个元素发布visualization_msgs/markerarray类型
*
*
*/

namespace map
{
struct RGBcolor
{
    double r_;//0~1
    double g_;
    double b_;
    RGBcolor() {}
    RGBcolor(double red, double green, double blue)
    {
        this->r_ = red;
        this->g_ = green;
        this->b_ = blue;
    }
    RGBcolor& operator=(const RGBcolor &a)
    {
        this->r_ = a.r_;
        this->g_ = a.g_;
        this->b_ = a.b_;
        return *this;
    }
};

enum class species
{
    EDGE = 0,
    NODE = 1,
    STOP_LINE = 1,
    PLAN_LINE = 2,
    CENTER = 3
};

class MapVisualization
{
private:
    //ros::NodeHandle n;
    //ros::Publisher map_pub;
    //ros::Publisher path_pub;
    //ros::Time currenttime;
    //visualization_msgs::MarkerArray map;
    //visualization_msgs::MarkerArray path;
    RGBcolor *colors_;
    //node可视化
    void Nodes2Marker(const node::Point3D *pin, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
    //way可视化
    void Ways2Marker(const node::Node *nodes, const way::Line *pin, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
    //道路中点可视化
    void Centerpoint2Marker(const centerway::CenterPoint3D *centerpoint, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
    //道路中心线
    void Centerway2Marker(const centerway::CenterWay *centerways, const centerway::CenterWay3D *centerway3ds, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
    //红绿灯
    void Redgreenlight2Marker(const node::Node *nodes, const way::Way *ways, const relation::relationship *relation, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
    //分割多边形为多个三角形
    bool IsWithinTriangle(const geometry_msgs::Point32 & a, const geometry_msgs::Point32 & b,
                          const geometry_msgs::Point32 & c, const geometry_msgs::Point32 & p) const;
    bool IsAcuteAngle(const geometry_msgs::Point32 & a, const geometry_msgs::Point32 & o,
                      const geometry_msgs::Point32 & b) const;
    void AdjacentPoints(const int i, const int N, const geometry_msgs::Polygon poly, geometry_msgs::Point32 * p0,
                        geometry_msgs::Point32 * p1, geometry_msgs::Point32 * p2) const;
    void Polygon2Triangle(const geometry_msgs::Polygon & polygon, std::vector<geometry_msgs::Polygon> * triangles) const;
    //人行道
    void Crosswalk2Marker(const node::Node *nodes, const way::Way *ways, const relation::relationship *crosswalk, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
public:
    MapVisualization();
    //地图可视化
    void Map2Marker(const node::Node *nodes, const way::Way *ways, const centerway::CenterWay *centerways, const relation::Relation *relations, visualization_msgs::MarkerArray &map, ros::Time nowtime) const;
    //规划路径可视化
    void Path2Marker(const centerway::CenterWay *centerways, std::vector<int> paths, visualization_msgs::MarkerArray &path, ros::Time nowtime) const;
    //路径平滑可视化
    void Smoothpath2Marker(const std::vector<map::centerway::CenterPoint3D> &smoothpath, visualization_msgs::MarkerArray &path, ros::Time nowtime) const;
    //清空pathmarkerarray
    ~MapVisualization();
};



};//namespace map

#endif