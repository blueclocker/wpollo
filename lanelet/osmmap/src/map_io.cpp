/*
 * @Author: your name
 * @Date: 2022-03-03 21:24:25
 * @LastEditTime: 2022-04-23 10:54:02
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_io.cpp
 */
#include "../include/osmmap/map_io.h"

namespace map
{

Map::Map(const std::string file_path_, const std::string file_name_):file_path(file_path_), file_name(file_name_)
{
    std::string file = file_path + file_name;
    TiXmlDocument doc;
    if(!doc.LoadFile(file.c_str()))
    {
        std::cout << "file input ERROR!" << std::endl;
        return;
    }
    TiXmlElement *ROOT = doc.FirstChildElement();
    if(ROOT == nullptr)
    {
        std::cout << "Failed to load file: No root element." << std::endl;
        doc.Clear();
        return;
    }
    node_pin = ROOT->FirstChildElement("node");
    way_pin = ROOT->FirstChildElement("way");
    relation_pin = ROOT->FirstChildElement("relation");

    //std::cout << "---------------------------------------------------" << std::endl;
    //node
    nodes = new node::Node(node_pin);
    nodes->CreateObjects(way_pin);
    std::cout << "nodes total number: " << nodes->Size() << std::endl;

    //way
    ways = new way::Way(way_pin);
    ways->CreateObjects(relation_pin);
    std::cout << "ways total number: " << ways->Size() << std::endl;
    
    //relation
    relations = new relation::Relation(relation_pin);
    relations->CreateObjects(nullptr);
    std::cout << "relations total number: " << relations->Size() << std::endl;
    
    //centerway
    centerways = new centerway::CenterWay(nullptr);
    centerways->run(nodes, ways, relations);
    std::cout << "centerways total number: " << centerways->Size() << std::endl;
    
    //output
    //std::cout << "************** map init successful!! **************" << std::endl;
    //std::cout << "---------------------------------------------------" << std::endl;
    doc.Clear();
    
}

void Map::setOrigin(const double lat_, const double lon_, const double ele_)
{
    geo_converter = new GeographicLib::LocalCartesian(lat_, lon_, ele_);
    origin_lat = lat_;
    origin_lon = lon_;
    origin_ele = ele_;
    std::cout << "origin has set already!" << std::endl;
}

node::Node const* Map::getNodesConstPtr() const
{
    return nodes;
}

way::Way const* Map::getWaysConstPtr() const 
{
    return ways;
}

relation::Relation const* Map::getRelationConstPtr() const
{
    return relations;
}

centerway::CenterWay const* Map::getCenterwayConstPtr() const
{
    return centerways;
}

void Map::GPS2Localxy(node::Point3D *node_) const
{
    geo_converter->Forward(node_->latitude, node_->longitude, node_->elevation,
                           node_->local_x, node_->local_y, node_->elevation);
}

Map::~Map()
{
    delete nodes;
    delete ways;
    delete relations;
    delete centerways;
    delete geo_converter;
}


};//namespace map