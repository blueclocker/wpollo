/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:36
 * @LastEditTime: 2022-04-22 14:46:56
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_io.h
 */
#ifndef MAP_IO_H_
#define MAP_IO_H_

#include "map_node.h"
#include "map_way.h"
#include "map_relation.h"
#include "centerway.h"
#include "GeographicLib/LocalCartesian.hpp"


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
    std::string file_path;
    std::string file_name;
    double origin_lat;
    double origin_lon;
    double origin_ele;

    TiXmlElement *node_pin;
    TiXmlElement *way_pin;
    TiXmlElement *relation_pin;
    
    node::Node *nodes;
    way::Way *ways;
    relation::Relation *relations;
    centerway::CenterWay *centerways;
    GeographicLib::LocalCartesian *geo_converter;

public:
    Map(const std::string file_path_, const std::string file_name_);
    ~Map();
    void setOrigin(const double lat_, const double lon_, const double ele_);
    node::Node const* getNodesConstPtr() const;
    way::Way const* getWaysConstPtr() const;
    relation::Relation const* getRelationConstPtr() const;
    centerway::CenterWay const* getCenterwayConstPtr() const;
    void GPS2Localxy(node::Point3D *node_) const;
};



};//namespace map

#endif