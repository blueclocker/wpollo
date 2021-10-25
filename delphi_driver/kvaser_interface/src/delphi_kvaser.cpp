#include "kvaser_interface/delphi_kvaser.h"

void DelphiKvaser::Callback(const can_msgs::Frame::ConstPtr &msg)
{
    float id, range, angle, rate, status, width, yaw_rate, vehicle_speed, moving, moving_fast, moving_slow, db, j;
    //std::vector<int> move;
    unsigned short a[64];
    //memset(&radar_target_data_,0,sizeof(radar_target_data_));
    //range 距离（m） angle角度（度） rate速度（m/s）width 宽度（m） x，y投影面坐标  
    //yaw_rate 车辆的横摆角速度 vehicle_speed 车速度
    int number = 1;


    //can报文
    if (msg->id == 0x4E0)
    {
        //CAN_TX_YAW_RATE_CALC
        unsigned short temp_A1 = msg->data[5];
        unsigned short temp_A2 = msg->data[6] & 0x00F0;
        unsigned short temp_A = temp_A1 & 0x0080;
        if (temp_A == 0)
        {
            yaw_rate = ((temp_A1 << 4) | (temp_A2)) * 0.0625f;
        }
        if (temp_A == 0x0080)
        {
            unsigned short temp_a0 = ((temp_A1 << 4) | temp_A2);
            unsigned short temp_a1 = ~temp_a0;
            unsigned short temp_a2 = (temp_a1 & 0x07FF) + 1;
            yaw_rate = -(temp_a2 * 0.0625f);
        }

        //CAN_TX_Vehicle_Speed_CALC
        temp_A1 = msg->data[6] & 0x0007;
        temp_A2 = msg->data[7];
        vehicle_speed = ((temp_A1 << 8) | (temp_A2)) * 0.0625f;

        //ROS_INFO("yaw_rate  %f \n", yaw_rate);
        //ROS_INFO("vehicle_speed  %f \n", vehicle_speed);
    }

 //动静模式
   else if(msg->id==0x540)
  {
      ////Group=7*9+1=10组
      unsigned short temp_A1 = msg->data[0];
      unsigned short temp_A2 = temp_A1&0x000F; //取出后四位
      if(temp_A2 != 9)
      {
        for(int j = 0; j < 7;++j)
        {
          //Moving 0=s 1=s
          unsigned short temp_D1 = (msg->data[j+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          moving = temp_D2;
          //fast
          unsigned short temp_S1 = (msg->data[j+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          moving_fast=temp_S2;
          //slow
          temp_S1 = (msg->data[j+1])&0x0040;
          temp_S2 = temp_S1>>6;
          moving_slow=temp_S2;
          //回波强度 -10到21dB
          temp_S1 = (msg->data[j+1])&0x001F;
          db=temp_S1-10;
          db_group[temp_A2 * 7 + j] = db;
          //cout << << endl;
        }
      }else{
          unsigned short temp_D1 = (msg->data[1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          moving=temp_D2;

          //fast
          unsigned short temp_S1 = (msg->data[1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          moving_fast=temp_S2;
          //slow
          temp_S1 = (msg->data[1])&0x0040;
          temp_S2 = temp_S1>>6;
          moving_slow=temp_S2;
          //回波强度
          temp_S1 = (msg->data[1])&0x001F;
          db=temp_S1-10;  
          db_group[63] = db;
        }
        //ROS_INFO("a  %f \n", a);
        //ROS_INFO("db  %f \n", db); 
       //end if(tmpCanID==0x540)
  } 

    else if(0x500 <= msg->id && msg->id <= 0x53F) //过滤帧，去64目标
    {

        id = msg->id - 1279;
        range = (((uint)msg->data[3]) + ((msg->data[2] & 0x0007) << 8)) * 0.1f;
        //angle = (((msg->data[1] & 0x000F) << 5) + ((msg->data[2] & 0x00F8) >> 3)) * 0.1f;
        unsigned short temp_A1 = msg->data[1];
        unsigned short temp_A2 = msg->data[2];
        unsigned short temp_A3 = temp_A1 & 0x0010;
        if (temp_A3 == 0)
        {
            angle = (((temp_A1 & 0x000F) << 5) | ((temp_A2 & 0x00F8) >> 3)) * 0.1f;
        }
        if (temp_A3 == 0x0010)
        {
            unsigned short temp_3 = ((temp_A1 & 0x000F) << 5) | ((temp_A2 & 0x00F8) >> 3);
            unsigned short temp_4 = ~temp_3;
            unsigned short temp_5 = (temp_4 & 0x01FF) + 1;
            angle = -temp_5 * 0.1f;
        }
        //rate = (msg->data[7] + ((msg->data[6] & 0x003F) << 8)) * 0.01f; //Unit: m/s;

        //range_rate  Unit: m/s
        unsigned short temp_V1 = msg->data[6];
        unsigned short temp_V2 = msg->data[7];
        unsigned short temp_V = temp_V1 & 0x0020;
        if (temp_V == 0)
        {
            rate = (temp_V2 | ((temp_V1 & 0x003F) << 8)) * 0.01f; //Unit: m/s
        }
        if (temp_V == 0x0020)
        {
            unsigned short temp_0 = ((temp_V1 & 0x003F) << 8) | temp_V2;
            unsigned short temp_1 = ~temp_0;
            unsigned short temp_2 = (temp_1 & 0x1FFF) + 1;
            rate = -(temp_2 * 0.01f); //Unit: m/s
        }

        //target width
        unsigned short temp_D1 = msg->data[4];
        width = (temp_D1 & 0x003C) * 0.5f;

        //蒙B长480mm，宽180mm p_x，p_y雷达点相对车中心点坐标
        double p_x_, p_y_;
        p_x_ = range * std::cos(angle*pi/180);
        p_y_ = range * std::sin(angle*pi/180);
        //u7，0无目标，1新目标，2，5，7保留目标，3更新目标，4滑动目标，6 无效目划行
        status = (msg->data[1] & 0x00E0 >> 5); 
        del_msg_.id = (int)id;
        del_msg_.angel = angle;
        del_msg_.range = range;
        del_msg_.rate = rate;
        del_msg_.width = width;
        //del_msg_.db = db;
        del_msg_.status = (int)status;
        //del_msg_.p_x = p_x_ + 2.4;//m
        //del_msg_.p_y = - p_y_;//m
        msg_pre.push_back(del_msg_);
        //visualization_msgs::MarkerArray
        msg_marker.header = msg -> header;
        msg_marker.header.frame_id = "delphi";
        msg_marker.id = (int)id;
        msg_marker.type = visualization_msgs::Marker::CUBE;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position.x = p_x_ + 2.4;
        msg_marker.pose.position.y = - p_y_;
        msg_marker.pose.position.z = 0;
        msg_marker.pose.orientation.x = 0;
        msg_marker.pose.orientation.y = 0;
        msg_marker.pose.orientation.z = 0;
        msg_marker.pose.orientation.w = 1;
        msg_marker.scale.x = 0.25;//width,m
        msg_marker.scale.y = 0.25;//length,m
        msg_marker.scale.z = 2;//height,m
        msg_marker.color.r = 0;
        msg_marker.color.g = 1;
        msg_marker.color.b = 0;
        msg_marker.color.a = 0.5;
        msg_marker.lifetime = ros::Duration(0);
        msg_marks.push_back(msg_marker);
    }
    if(msg_pre.size() == 64)
    {
        msg_out.header = msg->header;
        msg_out.header.frame_id = "delphi";
        for(int i = 0; i < 64; i++)
        {
          msg_pre[i].db = db_group[msg_pre[i].id - 1];
        }
        msg_out.delphi_msges = msg_pre;
        pub.publish(msg_out);
        msg_pre.clear();
        msg_markerarray.markers = msg_marks;
        pub_mark.publish(msg_markerarray);
        msg_marks.clear();
    }
    static tf::TransformBroadcaster broadcaster;
    tf::Transform base2radar;
    tf::Quaternion q;
    q.setRPY(0,0,M_PI);
    base2radar.setRotation(q);
    base2radar.setOrigin(tf::Vector3(2.4,0,0));
    broadcaster.sendTransform(tf::StampedTransform(base2radar,ros::Time::now(),"base_link","delphi"));
}

DelphiKvaser::DelphiKvaser(ros::NodeHandle &nh_)
{
    ros::Subscriber sub = nh_.subscribe("/can_tx", 10, &DelphiKvaser::Callback, this);
    pub = nh_.advertise<can_msgs::delphi_msges>("delphi_esr", 10); //发布雷达消息
    pub_mark = nh_.advertise<visualization_msgs::MarkerArray>("delphi_marker", 10); //发布雷达消息
    ros::spin();
}

DelphiKvaser::~DelphiKvaser()
{
}

