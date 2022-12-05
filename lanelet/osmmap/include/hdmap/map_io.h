/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:36
 * @LastEditTime: 2022-11-11 15:21:08
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/hdmap/map_io.h
 */
#ifndef MAP_IO_H_
#define MAP_IO_H_

#include <ctime>
#include <limits>
#include <chrono>
#include "map_node.h"
#include "map_way.h"
#include "map_relation.h"
#include "centerway.h"
// #include "tools/eigen2cv.hpp"
#include "GeographicLib/LocalCartesian.hpp"
// #include "grid_map_pcl/grid_map_pcl.hpp"
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/conditional_removal.h>
// #include "opencv2/core/core.hpp"
// #include "opencv2/core/eigen.hpp"
// #include "opencv2/opencv.hpp"


/*地图读取/可视化模块
* 调用node, way, relation, centerway四大类实现地图解析
* 
*/

namespace map
{
class Map
{
private:
    //params
    std::string file_path_;
    std::string file_name_;
    double origin_lat_;
    double origin_lon_;
    double origin_ele_;

    TiXmlElement *node_pin_;
    TiXmlElement *way_pin_;
    TiXmlElement *relation_pin_;
    
    node::Node *nodes_;
    way::Way *ways_;
    relation::Relation *relations_;
    centerway::CenterWay *centerways_;
    GeographicLib::LocalCartesian *geo_converter_;
    // grid_map::GridMap *gridmaps;
    // void tobinary(grid_map::GridMap::Matrix &data) const;

public:
    Map(const std::string file_path, const std::string file_name);
    ~Map();
    void SetOrigin(const double lat, const double lon, const double ele);
    node::Node const* GetNodesConstPtr() const;
    way::Way const* GetWaysConstPtr() const;
    relation::Relation const* GetRelationConstPtr() const;
    centerway::CenterWay const* GetCenterwayConstPtr() const;
    // grid_map::GridMap const* getGridmapConstPtr() const;
    void GPS2Localxy(node::Point3D *node) const;
};



};//namespace map

#endif