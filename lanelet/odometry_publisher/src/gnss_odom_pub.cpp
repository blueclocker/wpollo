#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h" 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
// #include <kvaser/CANPacket.h>
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
    //ros::Publisher gnss_lla_pub2;
    // ros::Publisher gnss_odometry_pub;
    int seq = 0;
    int gnss_seq = 0;
    int gnss_odo_seq = 0;
    Vector3d initial_lla;
    std::ofstream imu_file_;
    //tf::TransformBroadcaster odom_broadcaster;

    Eigen::Matrix3d Constant_Matrix;

    
  
  public:
    
    GPS():
    n("~"){
      Constant_Matrix << 1, 0, 0, 0, 0, 1, 0, -1, 0;
      gps_sub = n.subscribe("/comb", 1, &GPS::gpsCallback, this); 
      gnss_pub = n.advertise<fsd_common_msgs::Gnss>("/gnss_odom", 1);
      imu_pub = n.advertise<sensor_msgs::Imu>("/gnss_imu", 1);
      // imu_data_pub = n.advertise<sensor_msgs::Imu>("/imu/data",20);
      gnss_lla_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix",1);
      //gnss_lla_pub2 = n.advertise<sensor_msgs::NavSatFix>("/gps/fix2", 5);
      // gnss_odometry_pub = n.advertise<nav_msgs::Odometry>("/odometry/gps",5);
      imu_file_.open("/home/wangpeng/wpollo/src/odometry_publisher/imu_guochan.csv", std::ios::app);  
    }
    ~GPS() {}

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
    
    void gpsCallback(const fsd_common_msgs::comb::ConstPtr& comb){
      double latitude = comb->Lattitude;
      double longitude = comb->Longitude;
      double altitude = comb->Altitude;
      
      ros::Time current_time = comb->header.stamp;
      
      //n.getParam("param_a", param_a);
      //ROS_INFO("PARAM_A IS:%d", param_a);

      //read the gnss-imu data
      fsd_common_msgs::Gnss gnss;
      sensor_msgs::NavSatFix gnss_lla;
      sensor_msgs::NavSatFix gnss_lla2;

      gnss.status = comb->Status;
      gnss.PositionOK = comb->PositionOK;
      gnss.HeadingOK = comb->HeadingOK;
      gnss.WorkMode = comb->WorkMode;
      gnss.NoSV = comb->NoSV;
      gnss.header.stamp = comb->header.stamp;
      gnss.header.frame_id = "world";
      // read the LLA
      gnss.latitude = latitude;
      gnss.longitude = longitude;
      gnss.altitude = altitude;

      // read the RPY
      gnss.roll = (comb->Roll)*PI/180.0;
      gnss.pitch = (-comb->Pitch)*PI/180.0;
      gnss.yaw = comb->Heading*PI/180.0;

      // read the ENU velocity[m/s]
      gnss.vel_e = comb->Ve;
      gnss.vel_n = comb->Vn;
      gnss.vel_a = comb->Vu;

      ROS_INFO("pub imu gnss time is:%lf", current_time.toSec());

      // Publish the gnss topic
      gnss_pub.publish(gnss);
      
      //read the IMU data
      sensor_msgs::Imu gps_imu;
      gps_imu.header.stamp = comb->header.stamp;
      gps_imu.header.frame_id = "base_link";
      gps_imu.header.seq = ++seq;
      gps_imu.linear_acceleration.x = comb->Ax;
      gps_imu.linear_acceleration.y = -comb->Az;
      gps_imu.linear_acceleration.z = comb->Ay;
      gps_imu.angular_velocity.x = comb->Dx*PI/180.0;
      gps_imu.angular_velocity.y = -comb->Dz*PI/180.0; 
      gps_imu.angular_velocity.z = comb->Dy*PI/180.0;
      imu_file_.setf(std::ios::fixed, std::_S_floatfield);
      imu_file_ << comb->header.stamp.toSec() << ","
              << gps_imu.linear_acceleration.x << ","
              << gps_imu.linear_acceleration.y << ","
              << gps_imu.linear_acceleration.z << ","
              << gps_imu.angular_velocity.x << ","
              << gps_imu.angular_velocity.y << ","
              << gps_imu.angular_velocity.z << ","
              <<std::endl;
      gps_imu.orientation_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
      gps_imu.angular_velocity_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
      gps_imu.linear_acceleration_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
      

      double ENU_yaw = ((gnss.yaw+PI/2) > PI) ? gnss.yaw - 1.5 * PI : gnss.yaw + 0.5* PI;
      gps_imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(gnss.roll, gnss.pitch, ENU_yaw);

      // Eigen::Quaterniond temp = Eigen::Quaterniond(gps_imu.orientation.w, gps_imu.orientation.x,gps_imu.orientation.y,gps_imu.orientation.z);
      // Eigen::Quaterniond Q_final = Eigen::Quaterniond(Constant_Matrix)*temp;
      // gps_imu.orientation.w = Q_final.w();
      // gps_imu.orientation.x = Q_final.x();
      // gps_imu.orientation.y = Q_final.y();
      // gps_imu.orientation.z = Q_final.z();

      // Eigen::Vector3d eulerAngle=Q_final.matrix().eulerAngles(2,1,0);
      // cout<<"Trans Euler are: "<<eulerAngle(0)/M_PI*180<<" "<<eulerAngle(1)/M_PI*180<<" "<<eulerAngle(2)/M_PI*180<<endl;
      // cout<<"Trans Euler rad are: "<<eulerAngle(0)<<" "<<eulerAngle(1)<<" "<<eulerAngle(2)<<endl;

      cout<<"GNSS yaw is: "<<gnss.yaw<<endl<<"ENU_yaw is: "<<ENU_yaw<<endl<<endl;
      cout<<"pitch: "<< gnss.pitch<<endl;
      cout<<"roll: "<<gnss.roll<<endl;

      // Publish the imu topic
      imu_pub.publish(gps_imu);
      // imu_data_pub.publish(gps_imu);

      if(true)
      /*if(seq%5==0)*/ {
        gnss_lla.header.seq = ++gnss_seq;
        gnss_lla.header.stamp = comb->header.stamp;
        gnss_lla.header.frame_id = "imu_link";
        gnss_lla.latitude = latitude;
        gnss_lla.longitude = longitude;
        gnss_lla.altitude = altitude;
        // if(comb->Status==50) gnss_lla.position_covariance = {0.0001,0,0,0,0.0001,0,0,0,0.000225};
        // else if(comb->Status<50&&comb->Status>17) gnss_lla.position_covariance = {0.016,0,0,0,0.016,0,0,0,0.064};
        // else if(comb->Status<=17) gnss_lla.position_covariance = {2.25,0,0,0,2.25,0,0,0,9};
        gnss_lla.position_covariance = {gps_imu.orientation.w,gps_imu.orientation.x,gps_imu.orientation.y,gps_imu.orientation.z,0.0001,0,0,0,0.000225};
        // gnss_lla.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gnss_lla.COVARIANCE_TYPE_APPROXIMATED;
        gnss_lla.position_covariance_type = 1;
        gnss_lla_pub.publish(gnss_lla);

        gnss_lla2.header.seq = gnss_seq;
        gnss_lla2.header.stamp = comb->header.stamp;
        gnss_lla2.header.frame_id = "imu_link";
        gnss_lla2.latitude = latitude;
        gnss_lla2.longitude = longitude;
        gnss_lla2.altitude = altitude;
        // if(comb->Status==50) gnss_lla.position_covariance = {0.0001,0,0,0,0.0001,0,0,0,0.000225};
        // else if(comb->Status<50&&comb->Status>17) gnss_lla.position_covariance = {0.016,0,0,0,0.016,0,0,0,0.064};
        // else if(comb->Status<=17) gnss_lla.position_covariance = {2.25,0,0,0,2.25,0,0,0,9};
        // gnss_lla.position_covariance = {gps_imu.orientation.w,gps_imu.orientation.x,gps_imu.orientation.y,gps_imu.orientation.z,0.0001,0,0,0,0.000225};
        gnss_lla2.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gnss_lla2.COVARIANCE_TYPE_APPROXIMATED;
        gnss_lla2.position_covariance_type = 1;
        //gnss_lla_pub2.publish(gnss_lla2);



        nav_msgs::Odometry gnss_odo;
        if(gnss_odo_seq == 0&&comb->Status == 50){
          gnss_odo.header.seq = ++gnss_odo_seq;
          gnss_odo.header.stamp = comb->header.stamp;
          gnss_odo.header.frame_id = "odom";
          gnss_odo.pose.pose.position.x = 0;
          gnss_odo.pose.pose.position.y = 0;
          gnss_odo.pose.pose.position.z = 0;
          gnss_odo.pose.pose.orientation = gps_imu.orientation;
          gnss_odo.pose.covariance = {0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000225, 0.0, 0.0, 0.0, 
                                                                      0.0, 0.0, 0.0,0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076};
          initial_lla << latitude, longitude, altitude;
          cout<<gnss_odo.header.stamp<<endl;
          // gnss_odometry_pub.publish(gnss_odo);
        }
        else if(gnss_odo_seq != 0&& comb->Status == 50){
          gnss_odo.header.seq = ++gnss_odo_seq;
          gnss_odo.header.stamp = comb->header.stamp;
          gnss_odo.header.frame_id = "odom";

          Vector3d ENU;
          ConvertLLAToENU(initial_lla, Vector3d(latitude, longitude, altitude),&ENU);
          gnss_odo.pose.pose.position.x = ENU(0);
          gnss_odo.pose.pose.position.y = ENU(1);
          gnss_odo.pose.pose.position.z = ENU(2);
          gnss_odo.pose.pose.orientation = gps_imu.orientation;
          gnss_odo.pose.covariance = {0.0325*cos(latitude/180*PI)*cos(latitude/180*PI), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0325, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.04, 0.0, 0.0, 0.0, 
                                                                      0.0, 0.0, 0.0,0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0076};
          // gnss_odometry_pub.publish(gnss_odo);
        }
        else if(gnss_odo_seq != 0&& comb->Status<50&&comb->Status>17){
          gnss_odo.header.seq = ++gnss_odo_seq;
          gnss_odo.header.stamp = comb->header.stamp;
          gnss_odo.header.frame_id = "odom";

          Vector3d ENU;
          ConvertLLAToENU(initial_lla, Vector3d(latitude, longitude, altitude),&ENU);
          gnss_odo.pose.pose.position.x = ENU(0);
          gnss_odo.pose.pose.position.y = ENU(1);
          gnss_odo.pose.pose.position.z = ENU(2);
          gnss_odo.pose.pose.orientation = gps_imu.orientation;
          gnss_odo.pose.covariance = {0.425*cos(latitude/180*PI)*cos(latitude/180*PI), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.425, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.64, 0.0, 0.0, 0.0, 
                                                                      0.0, 0.0, 0.0,0.0152, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0152, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0152};
          // gnss_odometry_pub.publish(gnss_odo);
        }
        else if(gnss_odo_seq != 0&&comb->Status<=17){
          gnss_odo.header.seq = ++gnss_odo_seq;
          gnss_odo.header.stamp = comb->header.stamp;
          gnss_odo.header.frame_id = "odom";

          Vector3d ENU;
          ConvertLLAToENU(initial_lla, Vector3d(latitude, longitude, altitude),&ENU);
          gnss_odo.pose.pose.position.x = ENU(0);
          gnss_odo.pose.pose.position.y = ENU(1);
          gnss_odo.pose.pose.position.z = ENU(2);
          gnss_odo.pose.pose.orientation = gps_imu.orientation;
          gnss_odo.pose.covariance = {6.5*cos(latitude/180*PI)*cos(latitude/180*PI), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,9, 0.0, 0.0, 0.0, 
                                                                      0.0, 0.0, 0.0,0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05};
          // gnss_odometry_pub.publish(gnss_odo);
        }
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
