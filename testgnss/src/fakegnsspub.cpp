/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-14 21:52:30
 * @LastEditTime: 2022-09-14 22:30:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/testgnss/src/fakegnsspub.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <fsd_common_msgs/Comb.h>

void changeLLA(fsd_common_msgs::Comb &msg_)
{
    msg_.Lattitude += 0.00000001;
    msg_.Longitude += 0.00000001;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testGnss");
    ros::NodeHandle nh("~");

    ros::Publisher gnss_pub = nh.advertise<fsd_common_msgs::Comb>("/comb", 1);
    fsd_common_msgs::Comb msg;
    msg.header.frame_id = "gnss";
    msg.GPSTime = 0.0;
    msg.Heading = -91.7913894653;
    msg.Pitch = -3.407149533;
    msg.Roll = 0.28441759944;
    msg.Heading_std = 0.0;
    msg.Pitch_std = 0.0;
    msg.Roll_std = 0.0;
    msg.Lattitude = 39.955797;
    msg.Longitude = 116.3132292;
    msg.Altitude = 54.361541748;
    msg.Lattitude_std = 0.0;
    msg.Longitude_std = 0.0;
    msg.Altitude_std = 0.0;
    msg.Dx = 0.01689;
    msg.Dy = 0.1558759;
    msg.Dz = -0.69267;
    msg.Vn = -0.123828;
    msg.Vu = 0.00254;
    msg.Ve = 3.28297;
    msg.Ve_std = 0.0;
    msg.Vu_std = 0.0;
    msg.Vn_std = 0.0;
    msg.Ax = -0.5736506;
    msg.Ay = 9.8587;
    msg.Az = -0.051245;
    msg.Baseline = 0.0;
    msg.PositionOK = 1;
    msg.HeadingOK = 1;
    msg.WorkMode = 6;
    msg.NoSV = 25;
    msg.Angularacceleration = -1.0;
    msg.Status = 50;

    ros::Rate r(100);
    while(nh.ok())
    {
        msg.header.stamp = ros::Time::now();
        changeLLA(msg);
        gnss_pub.publish(msg);
        ROS_INFO("a gnss message publish!");

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

