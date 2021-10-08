#include<delphi/delphi.h>

void Callback(const can_msgs::Frame::ConstPtr &msg)
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

        //u7，0无目标，1新目标，2，5，7保留目标，3更新目标，4滑动目标，6 无效目划行
        status = (msg->data[1] & 0x00E0 >> 5); 
        del_msg_.id = (int)id;
        del_msg_.angel = angle;
        del_msg_.range = range;
        del_msg_.rate = rate;
        del_msg_.width = width;
        //del_msg_.db = db;
        del_msg_.status = (int)status;
        msg_pre.push_back(del_msg_);
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
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "delphi");
    // 创建节点句柄
    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/can_tx", 10, Callback);
    pub = nh.advertise<can_msgs::delphi_msges>("delphi_esr", 10); //发布雷达消息
    //ros::Rate loop_rate(1);
    ros::spin();
    return 0;
}
