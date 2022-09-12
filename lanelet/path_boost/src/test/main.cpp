/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-31 13:55:42
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-12 15:26:57
 * @FilePath: /wpollo/src/lanelet/path_boost/src/test/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <tuple>
#include <unistd.h>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros_viz_tools/ros_viz_tools.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_pcl/grid_map_pcl.hpp"
#include "glog/logging.h"
#include "eigen3/Eigen/Dense"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "path_boost/pathboost_core.h"
#include "path_boost/data_struct/basic_struct.hpp"
#include "path_boost/tools/tools.hpp"
#include "path_boost/data_struct/reference_path.hpp"
#include "path_boost/tools/spline.h"
#include "path_boost/tools/eigen2cv.hpp"
#include "path_boost/config/config_flags.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "osmmap/CarState.h"


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

void tobinary(grid_map::GridMap::Matrix &data)
{
    for(auto i = 0; i < data.rows(); ++i)
    {
        for(auto j = 0; j < data.cols(); ++j)
        {
            if(std::isnan(data(i, j)))
            {
                data(i, j) = 255;
            }else{
                data(i, j) = 0;
            }
        }
    }
}
PathBoostNS::BState start_state, end_state;
std::vector<PathBoostNS::BState> reference_path_plot;
PathBoostNS::ReferencePath reference_path_opt;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathboost");
    ros::NodeHandle nh;

    //log
    std::string base_dir = ros::package::getPath("path_boost");
    auto log_dir = base_dir + "/log";
    if (0 != access(log_dir.c_str(), 0)) {
        // if this folder not exist, create a new one.
        mkdir(log_dir.c_str(), 0777);
    }
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_log_dir = log_dir;
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;

    // Initialize grid map from pcd.
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
    grid_map::GridMap gridmap(std::vector<std::string>{"obstacle", "distance"});
    gridmap.setGeometry(gridMapPclLoader.getGridMap().getLength(), 
                        gridMapPclLoader.getGridMap().getResolution(), 
                        gridMapPclLoader.getGridMap().getPosition());
    // Add obstacle layer.
    auto obsdata = gridMapPclLoader.getGridMap().get("elevation");
    tobinary(obsdata);
    gridmap.add("obstacle", obsdata);
    // Update distance layer.
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
        gridmap.get("obstacle").cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(gridmap.get("distance")),
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
    gridmap.get("distance") *= 0.2;
    gridmap.setFrameId("/map");

    //publisher
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("bit_map", 1, true);
    // gridmap.setTimestamp(ros::Time::now().toNSec());
    // nav_msgs::OccupancyGrid message;
    // grid_map::GridMapRosConverter::toOccupancyGrid(gridmap, "obstacle", 255, 0, message);
    // map_publisher.publish(message);

    // Markers initialization.
    ros_viz_tools::RosVizTools markers(nh, "markers");
    std::string marker_frame_id = "/map";

    // Loop.
    ros::Rate rate(30.0);
    while (nh.ok())
    {
        ros::Time time = ros::Time::now();
        markers.clear();
        int id = 0;

        // Visualize reference path

        // Visualize start and end point selected by mouse.

        // Calculate.
        std::vector<PathBoostNS::SLState> result_path;
        std::vector<PathBoostNS::BState> smoothed_reference_path, result_path_by_boxes;
        std::vector<std::vector<double>> a_star_display(3);
        bool opt_ok = false;
        if (reference_rcv && start_state_rcv && end_state_rcv) {
            PathBoostNS::PathBoost path_optimizer(start_state, end_state, gridmap);
            opt_ok = path_optimizer.solve(reference_path_plot, &result_path);
            reference_path_opt = path_optimizer.getReferencePath();
            smoothed_reference_path.clear();
            if (!PathBoostNS::isEqual(reference_path_opt.getLength(), 0.0)) {
                double s = 0.0;
                while (s < reference_path_opt.getLength()) {
                    smoothed_reference_path.emplace_back(reference_path_opt.getXS()(s), reference_path_opt.getYS()(s));
                    s += 0.5;
                }
            }
            if (opt_ok) {
                std::cout << "ok!" << std::endl;

            }
        }

        // Visualize result path.
        ros_viz_tools::ColorRGBA path_color;
        path_color.r = 0.063;
        path_color.g = 0.305;
        path_color.b = 0.545;
        if (!opt_ok) {
            path_color.r = 1.0;
            path_color.g = 0.0;
            path_color.b = 0.0;
        }
        visualization_msgs::Marker result_marker =
            markers.newLineStrip(0.5, "optimized path", id++, path_color, marker_frame_id);
        for (size_t i = 0; i != result_path.size(); ++i) {
            geometry_msgs::Point p;
            p.x = result_path[i].x;
            p.y = result_path[i].y;
            p.z = 1.0;
            result_marker.points.push_back(p);
            const auto k = result_path[i].k;
            path_color.a = std::min(fabs(k) / 0.15, 1.0);
            path_color.a = std::max((float)0.1, path_color.a);
            result_marker.colors.emplace_back(path_color);
        }
        markers.append(result_marker);

        // Visualize result path.
        visualization_msgs::Marker result_boxes_marker =
            markers.newLineStrip(0.15, "optimized path by boxes", id++, ros_viz_tools::BLACK, marker_frame_id);
        for (size_t i = 0; i != result_path_by_boxes.size(); ++i) {
            geometry_msgs::Point p;
            p.x = result_path_by_boxes[i].x;
            p.y = result_path_by_boxes[i].y;
            p.z = 1.0;
            result_boxes_marker.points.push_back(p);
        }
        markers.append(result_boxes_marker);

        // Visualize smoothed reference path.
        visualization_msgs::Marker smoothed_reference_marker =
            markers.newLineStrip(0.07,
                                 "smoothed reference path",
                                 id++,
                                 ros_viz_tools::YELLOW,
                                 marker_frame_id);
        for (size_t i = 0; i != smoothed_reference_path.size(); ++i) {
            geometry_msgs::Point p;
            p.x = smoothed_reference_path[i].x;
            p.y = smoothed_reference_path[i].y;
            p.z = 1.0;
            smoothed_reference_marker.points.push_back(p);
        }
        markers.append(smoothed_reference_marker);
        ros_viz_tools::ColorRGBA vehicle_color = ros_viz_tools::GRAY;
        vehicle_color.a = 0.5;
        visualization_msgs::Marker vehicle_geometry_marker =
            markers.newLineList(0.03, "vehicle", id++, vehicle_color, marker_frame_id);
        // Visualize vehicle geometry.
        static const double length{FLAGS_car_length};
        static const double width{FLAGS_car_width};
        static const double rear_d{FLAGS_rear_length};
        static const double front_d{FLAGS_front_length};
        for (size_t i = 0; i != result_path.size(); ++i) {
            double heading = result_path[i].heading;
            PathBoostNS::BState p1, p2, p3, p4;
            p1.x = front_d;
            p1.y = width / 2;
            p2.x = front_d;
            p2.y = -width / 2;
            p3.x = rear_d;
            p3.y = -width / 2;
            p4.x = rear_d;
            p4.y = width / 2;
            auto tmp_relto = result_path[i];
            tmp_relto.heading = heading;
            p1 = PathBoostNS::local2Global(tmp_relto, p1);
            p2 = PathBoostNS::local2Global(tmp_relto, p2);
            p3 = PathBoostNS::local2Global(tmp_relto, p3);
            p4 = PathBoostNS::local2Global(tmp_relto, p4);
            geometry_msgs::Point pp1, pp2, pp3, pp4;
            pp1.x = p1.x;
            pp1.y = p1.y;
            pp1.z = 0.1;
            pp2.x = p2.x;
            pp2.y = p2.y;
            pp2.z = 0.1;
            pp3.x = p3.x;
            pp3.y = p3.y;
            pp3.z = 0.1;
            pp4.x = p4.x;
            pp4.y = p4.y;
            pp4.z = 0.1;
            vehicle_geometry_marker.points.push_back(pp1);
            vehicle_geometry_marker.points.push_back(pp2);
            vehicle_geometry_marker.points.push_back(pp2);
            vehicle_geometry_marker.points.push_back(pp3);
            vehicle_geometry_marker.points.push_back(pp3);
            vehicle_geometry_marker.points.push_back(pp4);
            vehicle_geometry_marker.points.push_back(pp4);
            vehicle_geometry_marker.points.push_back(pp1);
        }
        markers.append(vehicle_geometry_marker);

        // Publish the grid_map.
        gridmap.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(gridmap, "obstacle", 255, 0, message);
        map_publisher.publish(message);

        markers.publish();
        LOG_EVERY_N(INFO, 20) << "map published.";

        ros::spinOnce();
        rate.sleep();
    }

    // ros::spin();
    google::ShutdownGoogleLogging();
    return 0;
}


