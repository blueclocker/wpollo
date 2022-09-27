/*
 * @Author: your name
 * @Date: 2022-03-03 21:24:25
 * @LastEditTime: 2022-09-12 18:24:23
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/map_io.cpp
 */
#include "osmmap/map_io.h"

namespace map
{

Map::Map(const std::string file_path_, const std::string file_name_):file_path(file_path_), file_name(file_name_)
{
    const auto start = std::chrono::steady_clock::now();
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

    //init gridmaps
    // std::string pcdpath = file_path + "SurfMap.pcd";
    // std::string paramspath = file_path + "parameters.yaml";
    // //点云加载参数
    // grid_map::GridMapPclLoader gridMapPclLoader;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // if(pcl::io::loadPCDFile(pcdpath, *cloud) < 0)
    // {
    //     std::cout << "pcd file don't exist!" << std::endl;
    //     return;
    // }
    // //滤波
    // pcl::PointCloud<pcl::PointXYZ>::Ptr passcloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-4.0, 2.0);
    // pass.filter(*passcloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr removecloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // outrem.setInputCloud(passcloud);
    // outrem.setRadiusSearch(1.2);
    // outrem.setMinNeighborsInRadius (130);
    // outrem.filter(*removecloud);

    // gridMapPclLoader.setInputCloud(removecloud);
    // gridMapPclLoader.loadParameters(paramspath);

    // gridMapPclLoader.preProcessInputCloud();
    // gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
    // gridMapPclLoader.addLayerFromInputCloud("elevation");

    // gridmaps = new grid_map::GridMap(std::vector<std::string>{"obstacle", "distance"});
    // gridmaps->setGeometry(gridMapPclLoader.getGridMap().getLength(), 
    //                       gridMapPclLoader.getGridMap().getResolution(), 
    //                       gridMapPclLoader.getGridMap().getPosition());
    // // Add obstacle layer.
    // auto obsdata = gridMapPclLoader.getGridMap().get("elevation");
    // tobinary(obsdata);
    // gridmaps->add("obstacle", obsdata);
    // // Update distance layer.
    // Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
    //     gridmaps->get("obstacle").cast<unsigned char>();
    // cv::distanceTransform(eigen2cv(binary), eigen2cv(gridmaps->get("distance")),
    //                       CV_DIST_L2, CV_DIST_MASK_PRECISE);
    // gridmaps->get("distance") *= 0.2;
    // gridmaps->setFrameId("/map");

    const auto stop = std::chrono::steady_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
    std::cout << "init map takes " << duration << " ms." << std::endl;
    
}

// void Map::tobinary(grid_map::GridMap::Matrix &data) const
// {
//     for(auto i = 0; i < data.rows(); ++i)
//     {
//         for(auto j = 0; j < data.cols(); ++j)
//         {
//             if(std::isnan(data(i, j)))
//             {
//                 data(i, j) = 255;
//             }else{
//                 data(i, j) = 0;
//             }
//         }
//     }
// }

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

// grid_map::GridMap const* Map::getGridmapConstPtr() const
// {
//     return gridmaps;
// }

void Map::GPS2Localxy(node::Point3D *node_) const
{
    geo_converter->Forward(node_->latitude, node_->longitude, node_->elevation,
                           node_->local_x, node_->local_y, node_->elevation);
}

Map::~Map()
{
    // std::cout << "~Map" << std::endl;
    delete nodes;
    delete ways;
    delete relations;
    delete centerways;
    delete geo_converter;
    // delete gridmaps;
}


};//namespace map