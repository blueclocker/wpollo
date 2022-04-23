/*
 * @Author: your name
 * @Date: 2022-03-05 17:50:28
 * @LastEditTime: 2022-04-22 19:03:05
 * @LastEditors: Please set LastEditors
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

/*可视化模块，把个元素发布visualization_msgs/markerarray类型
*
*
*/

namespace map
{
struct RGBcolor
{
    double r;//0~1
    double g;
    double b;
    RGBcolor() {}
    RGBcolor(double red, double green, double blue)
    {
        this->r = red;
        this->g = green;
        this->b = blue;
    }
    RGBcolor& operator=(const RGBcolor &a)
    {
        this->r = a.r;
        this->g = a.g;
        this->b = a.b;
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
    RGBcolor *colors;
    //node可视化
    void nodes2marker(const node::Point3D *pin_, visualization_msgs::MarkerArray &map_, ros::Time nowtime_) const;
    //way可视化
    void ways2marker(const node::Node *nodes_, const way::Line *pin_, visualization_msgs::MarkerArray &map_, ros::Time nowtime_) const;
    //道路中点可视化
    void centerpoint2marker(const centerway::CenterPoint3D *centerpoint_, visualization_msgs::MarkerArray &map_, ros::Time nowtime_) const;
    //道路中心线
    void centerway2marker(const centerway::CenterWay *centerways_, const centerway::CenterWay3D *centerway3ds_, visualization_msgs::MarkerArray &map_, ros::Time nowtime_) const;
    //红绿灯
    void redgreenlight2marker(const node::Node *nodes_, const way::Way *ways_, const relation::relationship *relation_, visualization_msgs::MarkerArray &map_, ros::Time nowtime_) const;
public:
    MapVisualization();
    //地图可视化
    void map2marker(const node::Node *nodes_, const way::Way *ways_, const centerway::CenterWay *centerways_, const relation::Relation *relations_, visualization_msgs::MarkerArray &map_, ros::Time nowtime_) const;
    //规划路径可视化
    void path2marker(const centerway::CenterWay *centerways_, std::vector<int> paths_, visualization_msgs::MarkerArray &path_, ros::Time nowtime_) const;
    //路径平滑可视化
    void smoothpath2marker(const std::vector<map::centerway::CenterPoint3D> &smoothpath_, visualization_msgs::MarkerArray &path_, ros::Time nowtime_) const;
    //清空pathmarkerarray
    ~MapVisualization();
};



};//namespace map

#endif