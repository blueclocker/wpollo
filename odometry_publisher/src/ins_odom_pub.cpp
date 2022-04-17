#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h" 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <fsd_common_msgs/comb.h>
#include <fsd_common_msgs/Gnss.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <LocalCartesian.hpp>
#include <iostream>
#include <fstream>
#define PI 3.14159
#define Num 20

using namespace std;
using namespace Eigen;

class GPS{
  private:
    ros::NodeHandle n;
    
    ros::Subscriber gps_sub; 
    ros::Publisher gnss_pub;
    ros::Publisher imu_pub;
    // ros::Publisher imu_data_pub;
    ros::Publisher gnss_lla_pub;
    ros::Publisher gnss_odometry_pub;
    int seq = 0;
    int gnss_seq = 0;
    int gnss_odo_seq = 0;
    Vector3d initial_lla;
    //tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber imu_sub;
  
  public:
    
    GPS():
    n("~"){
        gps_sub = n.subscribe("/novatel/oem7/inspvax", 1, &GPS::gpsCallback, this);
        // imu_sub = n.subscribe("/imu/data", 1000, &GPS::imuCallback, this); 
        // imu_pub = n.advertise<sensor_msgs::Imu>("/gnss_imu", 5);
        // imu_data_pub = n.advertise<sensor_msgs::Imu>("/imu/data",20);
        gnss_lla_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix",5);
        gnss_odometry_pub = n.advertise<nav_msgs::Odometry>("/odometry/gps", 1000);
    }

    void ConvertLLAToENU(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_lla, 
                            Eigen::Vector3d* point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), 
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}

void ConvertENUToLLA(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_enu,
                            Eigen::Vector3d* point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), 
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);                            
}
    
    void imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){
      Eigen::Quaterniond Q(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z);
      Eigen::Vector3d v = Q.toRotationMatrix().eulerAngles(2, 1, 0).transpose() * 180 / M_PI;
      cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<endl;
    }

    void gpsCallback(const novatel_oem7_msgs::INSPVAXConstPtr& gpsRawMsg){
      double latitude = gpsRawMsg->latitude;
      double longitude = gpsRawMsg->longitude;
      double altitude = gpsRawMsg->height;
      double roll = gpsRawMsg->roll/180*M_PI;
      double pitch = gpsRawMsg->pitch/180*M_PI;
      double yaw = -gpsRawMsg->azimuth/180*M_PI;
      double ENU_yaw = ((yaw+PI/2) > PI) ? yaw - 1.5 * PI : yaw + 0.5* PI;
      
      ros::Time current_time = gpsRawMsg->header.stamp;
      
      //n.getParam("param_a", param_a);
      //ROS_INFO("PARAM_A IS:%d", param_a);

      //read the gnss data
      ROS_INFO("pub imu gnss time is:%lf", current_time.toSec());
      sensor_msgs::NavSatFix gnss_lla;
      
      if(true)
      /*if(seq%5==0)*/ {
        gnss_lla.header.seq = ++gnss_seq;
        gnss_lla.header.stamp = gpsRawMsg->header.stamp;
        gnss_lla.header.frame_id = "imu_link";
        gnss_lla.latitude = latitude;
        gnss_lla.longitude = longitude;
        gnss_lla.altitude = altitude;

        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, ENU_yaw);

        
        gnss_lla.position_covariance = {orientation.w,orientation.x,orientation.y,orientation.z,0,0,0,0,0};
        // gnss_lla.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gnss_lla.COVARIANCE_TYPE_APPROXIMATED;
        gnss_lla.position_covariance_type = 1;
        gnss_lla_pub.publish(gnss_lla);

        // nav_msgs::Odometry gnss_odo;
        // if(gnss_odo_seq == 0){
        //   gnss_odo.header.seq = ++gnss_odo_seq;
        //   gnss_odo.header.stamp = gpsRawMsg->header.stamp;
        //   gnss_odo.header.frame_id = "odom";
        //   gnss_odo.pose.pose.position.x = 0;
        //   gnss_odo.pose.pose.position.y = 0;
        //   gnss_odo.pose.pose.position.z = 0;
        //   gnss_odo.pose.pose.orientation = orientation;
        //   gnss_odo.pose.covariance = {0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000225, 0.0, 0.0, 0.0, 
        //                                                               0.0, 0.0, 0.0,0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076};
        //   initial_lla << latitude, longitude, altitude;
        //   cout<<gnss_odo.header.stamp<<endl;
        //   gnss_odometry_pub.publish(gnss_odo);
        // }
        // else if(gnss_odo_seq != 0){
        //   gnss_odo.header.seq = ++gnss_odo_seq;
        //   gnss_odo.header.stamp = gpsRawMsg->header.stamp;
        //   gnss_odo.header.frame_id = "odom";

        //   Vector3d ENU;
        //   ConvertLLAToENU(initial_lla, Vector3d(latitude, longitude, altitude),&ENU);
        //   gnss_odo.pose.pose.position.x = ENU(0);
        //   gnss_odo.pose.pose.position.y = ENU(1);
        //   gnss_odo.pose.pose.position.z = ENU(2);
        //   gnss_odo.pose.pose.orientation = orientation;
        //   gnss_odo.pose.covariance = {0.0325*cos(latitude/180*PI)*cos(latitude/180*PI), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0325, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.04, 0.0, 0.0, 0.0, 
        //                                                               0.0, 0.0, 0.0,0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076};
        //   gnss_odometry_pub.publish(gnss_odo);
        // }
      }
  }   
};

int main(int argc, char** argv){
  
  ros::init(argc, argv, "gnss_odom_pub");
  
  ROS_INFO("gnss ODOM pub code start !!!!");
  
  GPS ODOM;

  ros::spin();
    
  return 0;
}

