#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//#include <kvaser/CANPacket.h>
#include <fsd_common_msgs/e_comb.h>
#include <fsd_common_msgs/Gnss.h>
#include <math.h>
#include <cmath>
#define PI 3.14159
#define Num 20


class GPS{
  private:
    ros::NodeHandle n;
    
    ros::Subscriber gps_sub; 
    ros::Publisher gps_pub;
    ros::Publisher imu_pub;
    //tf::TransformBroadcaster odom_broadcaster;
  
  public:
    
    GPS():
    n("~"){
      gps_sub = n.subscribe("/e_comb", 1, &GPS::gpsCallback, this); 
      gps_pub = n.advertise<fsd_common_msgs::Gnss>("/gps_odom", 5);
      
    }
    
    void gpsCallback(const fsd_common_msgs::e_comb::ConstPtr& e_comb){
      
      ros::Time current_time = e_comb->header.stamp;
      
      //n.getParam("param_a", param_a);
      //ROS_INFO("PARAM_A IS:%d", param_a);

      //read the gps-imu data
      fsd_common_msgs::Gnss gps;

      gps.status = e_comb->Status;
      gps.header.stamp = e_comb->header.stamp;
      gps.header.frame_id = "world";
      // read the LLA
      gps.latitude = e_comb->Lattitude;
      gps.longitude = e_comb->Longitude;
      gps.altitude = e_comb->Altitude;

      // read the RPY
      gps.roll = (e_comb->Roll*PI)/180.0;
      gps.pitch = (-e_comb->Pitch) * PI/180.0;
      if(-e_comb->Heading < -180.0) 
	gps.yaw = (360.0-e_comb->Heading) * PI /180.0 ;
      else gps.yaw = -e_comb->Heading*PI/180.0 ;

      // read the ENU velocity[m/s]
      gps.vel_e = e_comb->Ve;
      gps.vel_n = e_comb->Vn;
      gps.vel_a = e_comb->Vu;
      
      ROS_INFO("pub imu gps time is:%lf", current_time.toSec());

      // Publish the gps topic
      gps_pub.publish(gps);
      
  }   
};

int main(int argc, char** argv){
  
  ros::init(argc, argv, "gps_odom_pub");
  
  ROS_INFO("gps ODOM pub code start !!!!");
  
  GPS ODOM;

  ros::spin();
    
  return 0;
}
/*
class GPS{
  private:
    ros::NodeHandle n;
    
    ros::Subscriber gps_sub; 
    ros::Publisher odom_pub;
    ros::Publisher imu_pub;
    tf::TransformBroadcaster odom_broadcaster;
    
    
    //variable
    double ori_De,ori_Dn,ori_Du,ori_Yaw;
    int first_gps = 0; 
    int param_a;
    
    double last_h = 0., last_p = 0., last_r = 0.;
    
  
  public:
    
    GPS():
    n("~"){
      gps_sub = n.subscribe("/comb", 1, &GPS::gpsCallback, this); 
      odom_pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 5);
      imu_pub = n.advertise<sensor_msgs::Imu>("/gps_imu", 5);
      
    }
    
    void gpsCallback(const gps_bag::comb::ConstPtr& comb){
      
      ros::Time current_time = ros::Time::now();
      //n.getParam("param_a", param_a);
      //ROS_INFO("PARAM_A IS:%d", param_a);
      
      double comb_Heading = comb->Heading - 90;
      if(comb_Heading > 180) comb_Heading -= 360;
	    
      if(first_gps == 0) {
	    ori_De = comb->De;
	    ori_Dn = comb->Dn;
	    ori_Du = comb->Du;
		    
	    ori_Yaw = comb_Heading  ;
	    first_gps = 2;
	    
      }
      
      //update gps angle
      double Roll = comb->Roll / 180.0 * PI;
      double Pitch = comb->Pitch / 180.0 * PI;
      double Yaw = ori_Yaw - comb_Heading;
      if(Yaw > 180) Yaw -= 360;
      if(Yaw < -180) Yaw +=360;
      Yaw = Yaw / 180.0 * PI;
      //ROS_INFO("Roll:%f,........Pitch:%f,........Yaw:%f,......time is:%lf", Roll, Pitch, Yaw, current_time);

      //update imu
      double w_yaw;
      double w_roll = (Roll - last_r) * 10;
      double w_pitch = (Pitch - last_p) * 10;
      if((Yaw > 3.0 && Yaw <= PI) && (last_h < -3.0 && last_h >= -PI)) w_yaw = ((Yaw - PI) - (PI + last_h)) * 10;
      else if((Yaw < -3.0 && Yaw >= -PI) && (last_h > 3.0 && last_h <= PI)) w_yaw = ((Yaw + PI) + (PI - last_h)) * 10;
      else w_yaw = (Yaw - last_h) * 10;
      ROS_INFO("last yaw is:%f,..now yaw is:%f,..w_yaw is:%f",last_h, Yaw, w_yaw);
      //double w_yaw = (Yaw - last_h) * 10;
      last_r = Roll;
      last_p = Pitch;
      last_h = Yaw;
      
      //update position
      double Dx = cos(ori_Yaw / 180.0 * PI) * (comb->De-ori_De) - sin(ori_Yaw / 180.0 * PI) * (comb->Dn-ori_Dn);
      double Dy = sin(ori_Yaw / 180.0 * PI) * (comb->De-ori_De) + cos(ori_Yaw / 180.0 * PI) * (comb->Dn-ori_Dn);
      double Dz = comb->Du - ori_Du;

      //update velocity
      double Vx = cos(ori_Yaw / 180 * PI) * comb->Ve - sin(ori_Yaw / 180 * PI) * comb->Vn;
      double Vy = sin(ori_Yaw / 180 * PI) * comb->Ve + cos(ori_Yaw / 180 * PI) * comb->Vn;
      double Vz = comb->Vu;
      
      //update accelaration
      double Ax = cos(ori_Yaw / 180 * PI) * comb->Ae - sin(ori_Yaw / 180 * PI) * comb->An;
      double Ay = sin(ori_Yaw / 180 * PI) * comb->Ae + cos(ori_Yaw / 180 * PI) * comb->An;
      double Az = comb->Au;
      //ROS_INFO("roll:%f,-------pitch:%f,---------yaw:%f",Roll,Pitch,Yaw);
      
      //publish imu
      sensor_msgs::Imu gps_imu;
      gps_imu.header.stamp = current_time;
      gps_imu.linear_acceleration.x = Ax * 10;
      gps_imu.linear_acceleration.y = Ay * 10;
      gps_imu.linear_acceleration.z = Az * 10;
      gps_imu.angular_velocity.x = w_roll;
      gps_imu.angular_velocity.y = w_pitch;
      gps_imu.angular_velocity.z = w_yaw;
      
      
      //publish odometry
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(Roll, Pitch, Yaw);
      //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(m_Roll/180.0f*PI,m_Pitch/180.0f*PI,m_Yaw/180.0f*PI);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "gpsodom";
      odom_trans.child_frame_id = "gpsbase_link";//for amcl
    
      odom_trans.transform.translation.x = Dx;
      odom_trans.transform.translation.y = Dy;
      odom_trans.transform.translation.z = Dz;
      odom_trans.transform.rotation = odom_quat;
      
      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "gpsodom";
      odom.child_frame_id = "gpsbase_link";
    
      //set the position
      odom.pose.pose.position.x = Dx;
      odom.pose.pose.position.y = Dy;
      odom.pose.pose.position.z = Dz;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.twist.twist.linear.x = Vx;
      odom.twist.twist.linear.y = Vy;
      odom.twist.twist.linear.z = Vz;
      
      //publishe the imu
      imu_pub.publish(gps_imu);
      //ROS_INFO("pub imu time is:%lf", current_time.toSec());
      
      //publish the message
      odom_pub.publish(odom);
      ROS_INFO("pub imu time is:%lf", current_time.toSec());
      //ROS_INFO("ODOM PUB IS");
  }   
};

int main(int argc, char** argv){
  
  ros::init(argc, argv, "odometry_pub");
  
  ROS_INFO("ODOM pub code start !!!!");
  
  GPS ODOM;

  //ros::Rate r(10);
  
  //while(ros::ok()){

    ros::spin();
    

   // r.sleep();
  //}
  return 0;
}

  void odomCallback(const gps_bag::comb::ConstPtr& comb)
  {
    ros::Time current_time;
    
    double = comb->Heading - 90;
    if(comb_Heading > 180) comb_Heading -= 360;
	  
    if(first_gps == 0) {
	  ori_De = comb->De;
	  ori_Dn = comb->Dn;
	  ori_Du = comb->Du;
		  
	  ori_Yaw = comb_Heading  ;
	  time_yu=2;
    }

    //update gps angle
    double Roll = comb->Roll / 180.0 * PI;
    double Pitch = comb->Pitch / 180.0 * PI;
    double Yaw = ori_Yaw - comb_Heading;
    if(Yaw > 180) Yaw -= 360;
    if(Yaw < -180) Yaw +=360;
    Yaw = Yaw / 180.0 * PI;

    //update position
    double Dx = cos(ori_Yaw / 180.0 * PI) * (comb->De-ori_De) - sin(ori_Yaw / 180.0 * PI) * (comb->Dn-ori_Dn);
    double Dy = sin(ori_Yaw / 180.0 * PI) * (comb->De-ori_De) + cos(ori_Yaw / 180.0 * PI) * (comb->Dn-ori_Dn);
    double Dz = comb->Du - ori_Du;

    //update velocity
    double Vx = cos(ori_Yaw) * comb->Ve - sin(ori_Yaw) * comb->Vn;
    double Vy = sin(ori_Yaw) * comb->Ve + cos(ori_Yaw) * comb->Vn;
    double Vz = comb->Vu;
   //ROS_INFO("roll:%f,-------pitch:%f,---------yaw:%f",Roll,Pitch,Yaw);
    
    //publish odometry
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Yaw);
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(m_Roll/180.0f*PI,m_Pitch/180.0f*PI,m_Yaw/180.0f*PI);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";//for amcl
  
    odom_trans.transform.translation.x = Dx;
    odom_trans.transform.translation.y = Dy;
    odom_trans.transform.translation.z = Dz;
    odom_trans.transform.rotation = odom_quat;
    
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
  
    //set the position
    odom.pose.pose.position.x = Dx;
    odom.pose.pose.position.y = Dy;
    odom.pose.pose.position.z = Dz;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.linear.y = Vy;
    odom.twist.twist.angular.z = Vz;

    //publish the message
    odom_pub.publish(odom);
  }




//void odomCallback1(const kvaser::CANPacket::ConstPtr can_pub_msg)
//{  
    //复制速度
   // vx=can_pub_msg->vel;
  //  vy=0;
    
    //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
    //ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str());
//}

//void imuCallback2(const sensor_msgs::Imu::ConstPtr &imu)
//{  
    //复制角速度
   // vth = imu->angular_velocity.z;
    
    //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
   // ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str());
//}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ROS_INFO("ODOM pub code start !!!!");

  ros::NodeHandle n;
  
  ros::Subscriber gps_sub = n.subscribe("/comb", 1, odomCallback3); //订阅gps话题
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/ev200/odom", 5);
  tf::TransformBroadcaster odom_broadcaster;

  //double x = 0.0;
 // double y = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
   // double current_th=th;
 // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
 // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
 // double delta_th = vth * dt;

 // x += delta_x;
 // y += delta_y;
 // th += delta_th;

 //double vth=(current_th-last_th)/180*PI/dt;

//calculate the mean of gps
/*double Sum_e,Sum_n,Sum_Heading_z,Sum_u,Sum_Pitch,Sum_Roll,Sum_Heading_w;
for(j=0;j<Num;j++)
{
Sum_e=Sum_e+De[j];
Sum_n=Sum_n+Dn[j];
Sum_u=Sum_u+Du[j];

//Sum_Heading_z=Sum_Heading+Heading[j];

//Sum_Heading_w=Sum_Heading_w+cos(Heading[j]/2);
Sum_Pitch=Sum_Pitch+Pitch[j];
Sum_Roll=Sum_Roll+Roll[j];
}
x=Sum_e/Num;
y=Sum_n/Num;
z=Sum_u/Num;

//m_Yaw_z=Sum_Heading_z/Num;
//m_Yaw_w=Sum_Heading_w/Num;
m_Pitch=Sum_Pitch/Num;
m_Roll=Sum_Roll/Num;
//if(m_Yaw>180)
//m_Yaw-=360;
//printf("%f\n",m_Yaw);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Yaw);
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(m_Roll/180.0f*PI,m_Pitch/180.0f*PI,m_Yaw/180.0f*PI);


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";//for amcl
    //odom_trans.child_frame_id = "base_footprint";//for cartograph

    odom_trans.transform.translation.x = Dx;
    odom_trans.transform.translation.y = Dy;
    odom_trans.transform.translation.z = Dz;
     odom_trans.transform.rotation = odom_quat;
    //odom_trans.transform.rotation.z = sqrt(1.0-m_Yaw_w*m_Yaw_w);
    //odom_trans.transform.rotation.w = m_Yaw_w;
    //odom_trans.transform.rotation.x = 0;
    //odom_trans.transform.rotation.y = 0; 

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = Dx;
    odom.pose.pose.position.y = Dy;
    odom.pose.pose.position.z = Dz;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link"; //for amcl
    //odom.child_frame_id = "base_footprint";//for cartograph
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.linear.y = Vy;
    odom.twist.twist.angular.z = Vz;

    //publish the message
    odom_pub.publish(odom);

   Sum_e=0;//initialize the Sum
    Sum_n=0;
    Sum_u=0;

    //Sum_Heading_z=0;
    //Sum_Heading_w=0;
    Sum_Pitch=0;
    Sum_Roll=0;
   
    last_time = current_time;
    //last_th=current_th;
    r.sleep();
  }
}

*/
