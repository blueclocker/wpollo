// Copyright (C) 2018  Zhi Yan and Li Sun

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
//#include "adaptive_clustering/ClusterArray.h"//自定义消息的头文件
#include <geometry_msgs/PolygonStamped.h>
#include <iostream>
//消息，盒子坐标输出
#include <geometry_msgs/Twist.h>
//#include "adaptive_clustering.h"
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
//就版本的发送消息
#include <rs_perception/PerceptionListMsg.h>
//#define LOG

// ros::Publisher cluster_array_pub_;
ros::Publisher cloud_filtered_pub_;
ros::Publisher pose_array_pub_;
ros::Publisher marker_array_pub_;
ros::Publisher box_array_pub_;
ros::Publisher msg_array_pub_;
bool print_fps_;
float z_axis_min_;
float z_axis_max_;
int cluster_size_min_;
int cluster_size_max_;
using namespace std;
const int region_max_ = 10; // Change this value to match how far you want to detect.半径·是10区域
int regions_[100];          // x*x+y*y+z*z<100

int frames;
clock_t start_time;
bool reset = true; // fps
//定义一个pointCloudCallback函数{}
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_pc2_in)
{
  if (print_fps_)
    if (reset)
    {
      frames = 0;
      start_time = clock();
      reset = false;
    } // fps  ros中点云ros_pc2_in  函数名字 pointCloudCallback

  /*** Convert ROS message to PCL ***/
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>);       //定义一个pcl点云指针pcl_pc_in
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);             //定义一个pcl点云指针pcl_pc_in
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_region(new pcl::PointCloud<pcl::PointXYZ>); //定义一个pcl点云指针pcl_pc_in
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);                                                // ros点云到pcl点云格式
  /*** 下采样
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(pcl_pc_in);
    vg.setLeafSize(0.15f, 0.15f, 0.15f);//double min_distance,0.2f, 0.2f, 0.2f
    vg.filter(*out);*/

  /*** Remove ground and ceiling  地面滤波。去除车身上点***/


  /*** Remove ground and ceiling  地面滤波。限定范围***/
  pcl::CropBox<pcl::PointXYZ> region(true);
  // Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
  // Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
  region.setMin(Eigen::Vector4f(-10, -10, -3.0, 1.0)); //-20, -7.5, -2, 1  -10, -5, -3.0, 1.0
  region.setMax(Eigen::Vector4f(30, 10, 1.0, 1.0));    // 30, 7.5, 1.2, 1       30, 5, 0, 1.0
  region.setInputCloud(pcl_pc_in);                     //输入体素out
  region.filter(*out);
  /*** Remove ground and ceiling  地面滤波。直通***/
  pcl::IndicesPtr pc_indices(new std::vector<int>); //创建一组索引，记录点云个数。指向int类·型的vector类空智能指针。
  pcl::PassThrough<pcl::PointXYZ> pt;               //直通滤波
  pt.setInputCloud(out);                            // pcl_pc_in
  pt.setFilterFieldName("z");
  pt.setFilterLimits(z_axis_min_, z_axis_max_); //-3 到 1米
  pt.filter(*pc_indices);                       //滤波后的点云是pc_indices
                                                /* */

  /*** Divide the point cloud into nested circular regions 分为11个区域，放在indices_array，点云到圆形区域***/
  boost::array<std::vector<int>, region_max_> indices_array; // boost是指针，array是数组 region_max_=10，定义11个vestor容器，放点云指针，所以int整形。
  for (int i = 0; i < pc_indices->size(); i++)               //开始历变滤波后的点云
  {
    float range = 0.0;
    for (int j = 0; j < region_max_; j++)
    {
      float d2 = out->points[(*pc_indices)[i]].x * out->points[(*pc_indices)[i]].x +
                 out->points[(*pc_indices)[i]].y * out->points[(*pc_indices)[i]].y +
                 out->points[(*pc_indices)[i]].z * out->points[(*pc_indices)[i]].z;  //求点的x*x+y*y+z*z之和给d2
      if (d2 > range * range && d2 <= (range + regions_[j]) * (range + regions_[j])) // regions_：0到13
      {
        indices_array[j].push_back((*pc_indices)[i]);
        break;
      }
      range += regions_[j];
    }
  }

  /*** Euclidean clustering ***/
  float tolerance = 0.0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> clusters; //定义了两个模班类 vector<数据类型> 名称 前一个是容器，后一个是内存分配，Allocator 负责提供 vector 需要用到的动态内存 点云指针的vector

  for (int i = 0; i < region_max_; i++)
  {

    tolerance += 0.4; //聚类半径
    if (indices_array[i].size() > cluster_size_min_)
    {
      boost::shared_ptr<std::vector<int>> indices_array_ptr(new std::vector<int>(indices_array[i])); //新建一个容器，boost::shared_ptr是Boost智能指针
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(out, indices_array_ptr); // indices_array_ptr每片区域内点云指针

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(out);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) //历变区域内索引cluster_indices，每个聚类的对象在进行以下历变
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>); //定义聚类后
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
          cluster->points.push_back(out->points[*pit]); //
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
      }
    }
  }

  /*** Output ***/
  if (cloud_filtered_pub_.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZ>); //定义输出点云集pcl_pc_out
    sensor_msgs::PointCloud2 ros_pc2_out;                                               //定义输出ros名 ros_pc2_out
    pcl::copyPointCloud(*out, *pc_indices, *pcl_pc_out);                                //把pcl_pc_in中点和pc_indices下表复制给pcl_pc_out
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);                                            //把pcl_pc_out格式转换为ros_pc2_out
    cloud_filtered_pub_.publish(ros_pc2_out);                                           //发布ros话题cloud_filtered_pub_
  }
  /****    划盒子**/
  // adaptive_clustering::ClusterArray cluster_array;//已经写好的聚类方法重命名cluster_array  pose_array   marker_array

  geometry_msgs::PoseArray pose_array; //中心点位姿
                                       /*geometry_msgs::PoseArray msg_array;//发送包围盒顶点坐标*/
  rs_perception::PerceptionListMsg rs_msg_array;
  rs_perception::PerceptionMsg msg_array; //发送包围盒顶点坐标

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray box_array;

  double time = ros::Time::now().toSec(); //把时间戳转化成浮点型格式
  msg_array.timestamp = time;
  rs_msg_array.header.stamp = ros::Time::now();
  rs_msg_array.header.frame_id = "map";
  /*  开始循环 */

  for (int i = 0; i < clusters.size(); i++)
  {
    /*
       if(cluster_array_pub_.getNumSubscribers() > 0)
          {
          sensor_msgs::PointCloud2 ros_pc2_out;
          pcl::toROSMsg(*clusters[i], ros_pc2_out);
          cluster_array.clusters.push_back(ros_pc2_out);
          }
    */
    /* if(pose_array_pub_.getNumSubscribers() > 0)
         {
         Eigen::Vector4f centroid;
         pcl::compute3DCentroid(*clusters[i], centroid);

         geometry_msgs::Pose pose;
         pose.position.x = centroid[0];
         pose.position.y = centroid[1];
         pose.position.z = centroid[2];
         pose.orientation.w = 1;

         pose_array.poses.push_back(pose);
    std::cout << pose.position.x << std::endl;

 #ifdef LOG
       Eigen::Vector4f min, max;
       pcl::getMinMax3D(*clusters[i], min, max);
       std::cerr << ros_pc2_in->header.seq << " "
     << ros_pc2_in->header.stamp << " "
     << min[0] << " "
     << min[1] << " "
     << min[2] << " "
     << max[0] << " "
     << max[1] << " "
     << max[2] << " "
     << std::endl;
 #endif
          }*/
    /*     3米的车道内的包围盒
     Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);*/
    // if(fabs(centroid[1])<10){
    /*     3米的车道内的包围盒   */

    if (marker_array_pub_.getNumSubscribers() > 0 || box_array_pub_.getNumSubscribers() > 0 || msg_array_pub_.getNumSubscribers() > 0)
    {

      Eigen::Vector4f min, max;
      pcl::getMinMax3D(*clusters[i], min, max);
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clusters[i], centroid);
      /*     包围盒中心坐标   */
      msg_array.centroid_x = centroid[0];
      msg_array.centroid_y = centroid[1];

      /* 凸包  */
      std::vector<cv::Point2f> points;
      for (int j = 0; j < clusters[i]->size(); ++j)
      {
        cv::Point2f pt;
        pt.x = clusters[i]->points[j].x;
        pt.y = clusters[i]->points[j].y;
        points.push_back(pt);
      }
      std::vector<cv::Point2f> hull;
      cv::convexHull(points, hull);
      geometry_msgs::PolygonStamped polygon;

      for (size_t j = 0; j < hull.size() + 1; j++)
      {
        geometry_msgs::Point32 point;
        point.x = hull[j % hull.size()].x;
        point.y = hull[j % hull.size()].y;
        point.z = min[2]; // z最小值
        polygon.polygon.points.push_back(point);
        /*save_marry_points[j].x=hull[j%hull.size()].x;
          save_marry_points[j].y=hull[j%hull.size()].y;
          del_msg_.x=0.0;
           del_msg_.y=0.0;
          msg_pre.push_back(del_msg_);*/
        ////***   发送包围盒顶点坐标       *******/////

        msg_array.id = i;
        msg_array.boxNum = clusters.size();      //一帧点云中盒子的总数量
        msg_array.boxpointNum = hull.size() + 1; //第i个盒子内凸包特征点数量
        msg_array.pointNum = i;                  //一帧点云中第i个盒子，从0计数

        geometry_msgs::Point points;
        points.x = point.x;
        points.y = point.y;
        msg_array.polygon_point.push_back(points);
      }
      rs_msg_array.perceptions.push_back(msg_array);

      msg_array.polygon_point.clear();

      for (size_t j = 0; j < hull.size() + 1; j++)
      {
        geometry_msgs::Point32 point;
        point.x = hull[j % hull.size()].x;
        point.y = hull[j % hull.size()].y;
        point.z = max[2]; // z最大值
        polygon.polygon.points.push_back(point);
      }

      visualization_msgs::Marker box;
      box.lifetime = ros::Duration(0.5);
      box.header = ros_pc2_in->header;

      box.type = visualization_msgs::Marker::LINE_STRIP;
      box.action = visualization_msgs::Marker::ADD;
      box.id = i;
      box.ns = "box";
      box.scale.x = 0.1;
      box.pose.orientation.x = 0.0;
      box.pose.orientation.y = 0.0;
      box.pose.orientation.z = 0.0;
      box.pose.orientation.w = 1.0;
      box.color.a = 0.5;
      box.color.g = 1.0;
      box.color.b = 0.0;
      box.color.r = 0.0;
      for (auto const &point : polygon.polygon.points)
      {
        geometry_msgs::Point tmp_point;
        tmp_point.x = point.x;
        tmp_point.y = point.y;
        tmp_point.z = point.z;

        box.points.push_back(tmp_point);
      }

      box_array.markers.push_back(box);

      visualization_msgs::Marker marker;
      marker.header = ros_pc2_in->header;
      marker.ns = "adaptive_clustering";
      marker.id = i;                                       //
      marker.type = visualization_msgs::Marker::LINE_LIST; //线性盒子

      geometry_msgs::Point p[hull.size() * 2]; // 12条
      for (int i = 0; i < hull.size() * 2; i = i + 2)
      {
        p[i].x = hull[i / 2].x;
        p[i].y = hull[i / 2].y;
        p[i].z = max[2];
        p[i + 1].x = hull[i / 2].x;
        p[i + 1].y = hull[i / 2].y;
        p[i + 1].z = min[2];
      }
      for (int i = 0; i < hull.size() * 2; i++)
      {
        marker.points.push_back(p[i]);
      }
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.color.a = 0.5;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(0.5);
      marker_array.markers.push_back(marker);
    }
    // }//3米的车道内的包围盒
  }

  /*
  if(cluster_array.clusters.size())
    {
     cluster_array.header = ros_pc2_in->header;
     cluster_array_pub_.publish(cluster_array);

    }
 */
  if (pose_array.poses.size())
  {
    pose_array.header = ros_pc2_in->header;
    pose_array_pub_.publish(pose_array);
  }

  if (rs_msg_array.perceptions.size())
  {
    rs_msg_array.header = ros_pc2_in->header;
    msg_array_pub_.publish(rs_msg_array);
  }

  if (marker_array.markers.size())
  {
    marker_array_pub_.publish(marker_array);
  }
  if (box_array.markers.size())
  {
    box_array_pub_.publish(box_array);
  }

  if (print_fps_)
    if (++frames > 10)
    {
      // std::cerr<<"[adaptive_clustering] fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<", points = "<< out->points.size()<<"循环！" <<std::endl;reset = true;
    } // fps
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptive_clustering");

  /*** Subscribers ***/
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("filtered_points_no_ground", 1, pointCloudCallback); // velodyne  rslidar   cloud_add_points
  std::cerr << "adaptive_clustering start!" << std::endl;
  /*** Publishers ***/
  ros::NodeHandle private_nh("~"); //命名空间/adaptive_clustering
                                   // cluster_array_pub_ = private_nh.advertise<adaptive_clustering::ClusterArray>("clusters", 100);
  cloud_filtered_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
  // pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
  msg_array_pub_ = private_nh.advertise<rs_perception::PerceptionListMsg>("rs_percept_result", 1); //发送顶点坐标
  box_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers_tubao", 1);      //包围盒
  marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers_line", 1);    //包围盒
                                                                                                   // ros::Rate loop_rate(15);
  /*** Parameters ***/
  std::string sensor_model;

  private_nh.param<std::string>("sensor_model", sensor_model, "HDL-32E"); // VLP-16, HDL-32E, HDL-64E
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<float>("z_axis_min", z_axis_min_, -3);
  private_nh.param<float>("z_axis_max", z_axis_max_, 1.0);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 50); // 15
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 2200);

  // Divide the point cloud into nested circular regions centred at the sensor.
  // For more details, see our IROS-17 paper "Online learning for human classification in 3D LiDAR-based tracking"
  if (sensor_model.compare("VLP-16") == 0)
  {
    regions_[0] = 2;
    regions_[1] = 3;
    regions_[2] = 3;
    regions_[3] = 3;
    regions_[4] = 3;
    regions_[5] = 3;
    regions_[6] = 3;
    regions_[7] = 2;
    regions_[8] =
        3;
    regions_[9] = 3;
    regions_[10] = 3;
    regions_[11] = 3;
    regions_[12] = 3;
    regions_[13] = 3;
  }
  else if (sensor_model.compare("HDL-32E") == 0)
  {
    regions_[0] = 100;
    regions_[1] = 10;
    regions_[2] = 10;
    regions_[3] = 10;
    regions_[4] = 10;
    regions_[5] = 10;
    regions_[6] = 10;
    regions_[7] = 10;
    regions_[8] = 10;
    regions_[9] = 10;
    regions_[10] = 5;
    regions_[11] = 5;
    regions_[12] = 4;
    regions_[13] = 5; // 4 5  4 5 4 5 5 4 5 4
  }
  else if (sensor_model.compare("HDL-64E") == 0)
  {
    regions_[0] = 14;
    regions_[1] = 14;
    regions_[2] = 14;
    regions_[3] = 15;
    regions_[4] = 14;
  }
  else
  {
    ROS_FATAL("Unknown sensor model!");
  }

  ros::spin();

  return 0;
}

/**** msg_array_pub_为给规划的障碍物凸包尺寸，二维的x，y
记得source*/
////
