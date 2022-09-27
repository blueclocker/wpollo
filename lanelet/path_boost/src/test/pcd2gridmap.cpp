/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-01 15:20:40
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-12 21:33:53
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
#include "hastar/hybidacore.h"
#include "path_boost/tools/eigen2cv.hpp"
#include <tf/tf.h>
#include <nav_msgs/Path.h>

HybidA::Vec3d start_state, goal_state;
ros::Publisher map_publisher;
ros::Publisher path_pub;
grid_map::GridMap gridmap(std::vector<std::string>{"obstacle", "distance"});
nav_msgs::OccupancyGrid message;
bool start_state_rcv = false, end_state_rcv = false, has_map = false;
std::shared_ptr<HybidA::hybidacore> kinodynamic_astar_searcher_ptr_;

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

void PublishPath(const VectorVec3d &path) 
{
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time::now();

    path_pub.publish(nav_path);
}

void carstatecallback(const osmmap::CarState::ConstPtr &msgptr)
{
    // bool subflag = false;
    int x = msgptr->carPose.position.x;
    int y = msgptr->carPose.position.y;
    start_state.x() = x;
    start_state.y() = y;
    start_state.z() = tf::getYaw(msgptr->carPose.orientation);
    start_state_rcv = true;
    goal_state.x() = msgptr->endPose.position.x;
    goal_state.y() = msgptr->endPose.position.y;
    goal_state.z() = tf::getYaw(msgptr->endPose.orientation);
    if(goal_state.x() != 0 && goal_state.y() != 0)
    {
        end_state_rcv = true;
    }
    // grid_map::GridMap submap = gridMap.getSubmap({x, y}, {50, 30}, subflag);
    // float a = gridMap.atPosition("elevation", {0, 0}, grid_map::InterpolationMethods::INTER_LINEAR);
    // std::cout << "now=" << a << std::endl;
    // grid_map::Length len = submap.getLength();
    // grid_map::Position pos = submap.getPosition();
    // submap.setGeometry(len, 0.2, pos);
    // submap.setFrameId("/map");

    // 改分辨率
    // grid_map::GridMap modifiedmap;
    // grid_map::GridMapCvProcessing::changeResolution(submap, modifiedmap, 0.2);
    
    // nav_msgs::OccupancyGridPtr message;
    // grid_map::GridMapRosConverter::toOccupancyGrid(modifiedmap, "elevation", 255, 0, message);
    // grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "elevation", 255, 0, *message);
    if(!has_map)
    {
      kinodynamic_astar_searcher_ptr_->init(message, 0.2);
      has_map = true; 
    }
    if (start_state_rcv && end_state_rcv && has_map)
    {
      if(kinodynamic_astar_searcher_ptr_->search(start_state, goal_state))
      {
        std::cout << "find" << std::endl;
        auto path = kinodynamic_astar_searcher_ptr_->getPath();
        PublishPath(path);
      }
    }
    // map_publisher.publish(*message);
}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    goal_state.x() = goal->pose.position.x;
    goal_state.y() = goal->pose.position.y;
    goal_state.z() = tf::getYaw(goal->pose.orientation);
    end_state_rcv = true;
    std::cout << "get the goal." << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd2gridmap");
    ros::NodeHandle nh("~");

    double steering_angle = nh.param("steering_angle", 10);
    int steering_angle_discrete_num = nh.param("steering_angle_discrete_num", 1);
    double wheel_base = nh.param("wheel_base", 1.0);
    double segment_length = nh.param("segment_length", 1.6);
    int segment_length_discrete_num = nh.param("segment_length_discrete_num", 8);
    double steering_penalty = nh.param("steering_penalty", 1.05);
    double steering_change_penalty = nh.param("steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("reversing_penalty", 2.0);
    double shot_distance = nh.param("shot_distance", 5.0);
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybidA::hybidacore>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );

    // ros::Publisher gridMapPub;
    // gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map_from_raw_pointcloud", 1, true);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);

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

    // grid_map::GridMap gridmap(std::vector<std::string>{"obstacle", "distance"});
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
    
    gridmap.setTimestamp(ros::Time::now().toNSec());
    // nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(gridmap, "obstacle", 255, 0, message);
    // grid_map::GridMapRosConverter::toOccupancyGrid(gridmap, "obstacle", 255, 0, message);

    // ros::Subscriber end_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);
    // ros::Subscriber followmap = nh.subscribe("/mapio/carstate_info", 1, &carstatecallback);


    ros::Rate r(10);
    while(nh.ok())
    {
        message.header.stamp = ros::Time::now();
        map_publisher.publish(message);
        r.sleep();
        ros::spinOnce();
    }

    // ros::Subscriber followmap = nh.subscribe("/mapio/carstate_info", 1, &carstatecallback);
    return 0;
}


