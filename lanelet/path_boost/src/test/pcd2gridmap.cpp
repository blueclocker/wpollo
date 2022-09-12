/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-01 15:20:40
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-04 15:21:34
 * @FilePath: /wpollo/src/lanelet/path_boost/src/test/pcd2gridmap.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * 
 */
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "grid_map_pcl/grid_map_pcl.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include "osmmap/CarState.h"

ros::Publisher map_publisher;
grid_map::GridMap gridMap;

void tooccupancymap(const grid_map::GridMap& gridMap,
                    const std::string& layer, float dataMin, float dataMax,
                    nav_msgs::OccupancyGrid& occupancyGrid)
{
//   std::cout << "tooccupancymap" << std::endl;
  occupancyGrid.header.frame_id = gridMap.getFrameId();
  occupancyGrid.header.stamp.fromNSec(gridMap.getTimestamp());
  occupancyGrid.info.map_load_time = occupancyGrid.header.stamp;  // Same as header stamp as we do not load the map.
  occupancyGrid.info.resolution = gridMap.getResolution();
  occupancyGrid.info.width = gridMap.getSize()(0);
  occupancyGrid.info.height = gridMap.getSize()(1);
  grid_map::Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  occupancyGrid.info.origin.position.x = position.x();
  occupancyGrid.info.origin.position.y = position.y();
  occupancyGrid.info.origin.position.z = 0.0;
  occupancyGrid.info.origin.orientation.x = 0.0;
  occupancyGrid.info.origin.orientation.y = 0.0;
  occupancyGrid.info.origin.orientation.z = 0.0;
  occupancyGrid.info.origin.orientation.w = 1.0;
  size_t nCells = gridMap.getSize().prod();
  occupancyGrid.data.resize(nCells);

  // Occupancy probabilities are in the range [0,100]. Unknown is -1.
  const float cellMin = 0;
  const float cellMax = 100;
  const float cellRange = cellMax - cellMin;

  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
    if (std::isnan(value))
      value = 0;
    else
      value = 100 - cellMin - std::min(std::max(0.0f, value), 1.0f) * cellRange;
    size_t index = grid_map::getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
    // Reverse cell order because of different conventions between occupancy grid and grid map.
    occupancyGrid.data[nCells - index - 1] = value;
  }
}


void carstatecallback(const osmmap::CarState::ConstPtr &msgptr)
{
    bool subflag = false;
    int x = msgptr->carPose.position.x;
    int y = msgptr->carPose.position.y;
    grid_map::GridMap submap = gridMap.getSubmap({x, y}, {50, 30}, subflag);
    float a = gridMap.atPosition("elevation", {0, 0}, grid_map::InterpolationMethods::INTER_LINEAR);
    std::cout << "now=" << a << std::endl;
    // grid_map::Length len = submap.getLength();
    // grid_map::Position pos = submap.getPosition();
    // submap.setGeometry(len, 0.2, pos);
    submap.setFrameId("/map");
    grid_map::GridMap modifiedmap;
    grid_map::GridMapCvProcessing::changeResolution(submap, modifiedmap, 0.2);
    
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(modifiedmap, "elevation", 0, 255, message);
    // grid_map::GridMapRosConverter::toOccupancyGrid(submap, "elevation", 0, 255, message);
    map_publisher.publish(message);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd2gridmap");
    ros::NodeHandle nh("~");

    ros::Publisher gridMapPub;
    gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map_from_raw_pointcloud", 1, true);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

    //点云加载参数
    grid_map::GridMapPclLoader gridMapPclLoader;
    const std::string pathtocloud = "/home/wangpeng/wpollo/src/lanelet/osmmap/maps/SurfMap.pcd";
    const std::string pathtoparameters = "/home/wangpeng/wpollo/src/lanelet/path_boost/config/parameters.yaml";
    
    //读取点云
    // gridMapPclLoader.loadCloudFromPcdFile(pathtocloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(pathtocloud, *cloud) < 0)
    {
        ROS_ERROR("pcd file don't exist!");
        return -1;
    }
    //滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr passcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-4.0, 2.0);
    pass.filter(*passcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr removecloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(passcloud);
    outrem.setRadiusSearch(1.2);
    outrem.setMinNeighborsInRadius (130);
    outrem.filter(*removecloud);

    gridMapPclLoader.setInputCloud(removecloud);
    gridMapPclLoader.loadParameters(pathtoparameters);

    grid_map::grid_map_pcl::processPointcloud(&gridMapPclLoader, nh);
    //以下等同上一句
    // const auto start = std::chrono::high_resolution_clock::now();
    // gridMapPclLoader->preProcessInputCloud();
    // gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
    // printTimeElapsedToRosInfoStream(start, "Initialization took: ");
    // gridMapPclLoader->addLayerFromInputCloud(getMapLayerName(nh));
    // printTimeElapsedToRosInfoStream(start, "Total time: ");

    gridMap = gridMapPclLoader.getGridMap();
    gridMap.setFrameId("/map");
    // float a = gridMap.atPosition("elevation", {0, 0}, grid_map::InterpolationMethods::INTER_CUBIC);
    // std::cout << "a=" << a << std::endl;
    bool subflag = false;
    grid_map::GridMap submap = gridMap.getSubmap({100, 10}, {50, 30}, subflag);
    gridMap.setFrameId("/map");
    submap.setFrameId("/map");
    grid_map_msgs::GridMap msg;
    if(subflag)
    {
        std::cout << "submap succeed" << std::endl;
        grid_map::GridMapRosConverter::toMessage(submap, msg);
        
    }else{
        std::cout << "submap failed" << std::endl;
        grid_map::GridMapRosConverter::toMessage(gridMap, msg);
    }
    
    gridMap.setTimestamp(ros::Time::now().toNSec());
    nav_msgs::OccupancyGrid message;
    // grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "elevation", 0, 255, message);
    tooccupancymap(gridMap, "elevation", 0, 255, message);
    map_publisher.publish(message);

    ros::Rate r(10);
    while(nh.ok())
    {
        msg.info.header.stamp = ros::Time::now();
        gridMapPub.publish(msg);
        r.sleep();
        ros::spinOnce();
    }

    // ros::Subscriber followmap = nh.subscribe("/mapio/carstate_info", 1, &carstatecallback);
    ros::spin();
    return 0;
}


