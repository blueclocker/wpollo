#include <ros/ros.h>
#include <serial/serial.h>
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include "fsd_common_msgs/comb.h"
#include "fsd_common_msgs/state_as.h"
#include <stdio.h>

fsd_common_msgs::state_as state_as; // state_as means attached_struct
serial::Serial ser;
const int frequency = 100;
const int Baud_rate = 115200;
const int delay = 3;
union type_float {
  unsigned char store[4];
  float result;
};
union type_int {
  unsigned char store[4];
  int result;
};
union type_short {
  unsigned char store[2];
  short int result;
};
int main(int argc, char **argv) {
  //初始化节点
  ros::init(argc, argv, "serial_gps");
  //声明节点句柄
  ros::NodeHandle nh;

  ros::Publisher read_pub = nh.advertise<fsd_common_msgs::comb>("comb", 1000);
  ros::Publisher as_pub = nh.advertise<fsd_common_msgs::state_as>("state_as", 1000);

  try {
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(Baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(delay);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  //检测串口是否已经打开，并给出提示信息
  if (ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port initializced");
  } else {
    return -1;
  }
  std::string ret_temp1;
  std::string ret_temp2;
  //指定循环的频率
  ros::Rate loop_rate(frequency);
  while (ros::ok()) {
    if (ser.available()) {
      ROS_INFO_STREAM("Reading from serial port\n");
      std_msgs::String result;
      result.data = ser.readline();
      std::string ret = result.data;
      std::cout << "received:" << ret.length() << std::endl;
      bool flag = false;
      for (int i = 0; i < ret.length(); i++) {
        if (int(ret[i]) == 85 && int(ret[i + 1] == 170)) std::cout << "OK";
        //std::cout<<int(ret[i]);
      }
      std::cout << std::endl;
      if (ret.length() != 80) {
        ret_temp2 = ret_temp1 + ret;
        if (ret_temp2.length() != 80) {
          std::cout << "ret_temp2:" << ret_temp2.length() << std::endl;
          ret_temp1 = ret;
          continue;
        } else {
          result.data = ret_temp2;
          ret_temp1 = ret;
        }
      }
      fsd_common_msgs::comb pub;

      type_float temp_float[13];
      type_int temp_int[2], TimeCnt;
      type_short temp_short;
      unsigned char data[80];
      const char *tempp = result.data.c_str();
      for (int i = 0; i < 80; i++) {
        data[i] = *(tempp + i);
      }
      std::cout << int(data[2]) << std::endl;
      unsigned char sum = 0;
      for (int i = 2; i < 79; i++)
        sum = data[i] + sum;
      unsigned char check = (unsigned char) data[79];
	
      //std::cout<<int(sum)<<"ok"<<int(check)<<std::endl;
      //std::cout<<int(data[0])<<"yu"<<int(data[1])<<"dajfuah"<<int(data[2])<<std::endl;
      if (int(sum) == int(check) && int(data[0]) == 85 && int(data[1]) == 170 && int(data[2]) == 76) {
        std::cout << "ok" << std::endl;
        ROS_INFO_STREAM("Serial port of Solution has started\n");
        TimeCnt.store[0] = data[3];
        TimeCnt.store[1] = data[4];
        TimeCnt.store[2] = data[5];
        TimeCnt.store[3] = data[6];
        int mod = TimeCnt.result % 200;
        switch (mod) {
          type_int temp_asi;
          type_float temp_asf;
          case 0:state_as.WorkMode = data[7];
            temp_asi.store[0] = 0;
            temp_asi.store[1] = data[8];
            temp_asi.store[2] = data[9];
            temp_asi.store[3] = data[10];
            state_as.SelfTest = temp_asi.result;
            break;
          case 10:state_as.PosType = data[7];
            state_as.PositionOK = data[8];
            state_as.HeadingOK = data[9];
            state_as.NoSV = data[10];
            break;
          case 20:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.Temperature = temp_asf.result;
            break;
          case 30:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.HDOP = temp_asf.result;
            break;
          case 40:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.Vn0 = temp_asf.result;
            break;
          case 50:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.Vn1 = temp_asf.result;
            break;
          case 60:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.Vn2 = temp_asf.result;
            break;
          case 70:temp_asi.store[0] = data[7];
            temp_asi.store[1] = data[8];
            temp_asi.store[2] = data[9];
            temp_asi.store[3] = data[10];
            state_as.Latitude = temp_asi.result;
            break;
          case 80:temp_asi.store[0] = data[7];
            temp_asi.store[1] = data[8];
            temp_asi.store[2] = data[9];
            temp_asi.store[3] = data[10];
            state_as.Longtitude = temp_asi.result;
            break;
          case 90:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.Height = temp_asf.result;
            break;
          case 100:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.GyrDrift0 = temp_asf.result;
            break;
          case 110:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.GyrDrift1 = temp_asf.result;
            break;
          case 120:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.GyrDrift2 = temp_asf.result;
            break;
          case 130:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.PreHeight = temp_asf.result;
            break;
          case 140:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.MagHeading = temp_asf.result;
            break;
          case 150:temp_asf.store[0] = data[7];
            temp_asf.store[1] = data[8];
            temp_asf.store[2] = data[9];
            temp_asf.store[3] = data[10];
            state_as.GpsHeading = temp_asf.result;
            break;
          case 160:temp_asi.store[0] = data[7];
            temp_asi.store[1] = data[8];
            temp_asi.store[2] = data[9];
            temp_asi.store[3] = data[10];
            state_as.PPS_TIME = temp_asi.result;
            break;
          case 170:temp_asi.store[0] = data[7];
            temp_asi.store[1] = data[8];
            temp_asi.store[2] = data[9];
            temp_asi.store[3] = data[10];
            state_as.iTOW = temp_asi.result;
            break;
          default:break;
        }
        for (int i = 0; i < 10; i++) {
          temp_float[i].store[0] = data[11 + 4 * i];
          temp_float[i].store[1] = data[12 + 4 * i];
          temp_float[i].store[2] = data[13 + 4 * i];
          temp_float[i].store[3] = data[14 + 4 * i];
        }
        for (int i = 0; i < 2; i++) {
          temp_int[i].store[0] = data[51 + 4 * i];
          temp_int[i].store[1] = data[52 + 4 * i];
          temp_int[i].store[2] = data[53 + 4 * i];
          temp_int[i].store[3] = data[54 + 4 * i];
        }
        for (int i = 0; i < 3; i++) {
          temp_float[i + 10].store[0] = data[59 + 4 * i];
          temp_float[i + 10].store[1] = data[60 + 4 * i];
          temp_float[i + 10].store[2] = data[61 + 4 * i];
          temp_float[i + 10].store[3] = data[62 + 4 * i];
        }
        temp_short.store[0] = data[77];
        temp_short.store[1] = data[78];
        pub.header.stamp = ros::Time::now();
        pub.header.frame_id = "velodyne";
        pub.GPSTime = 0;
        pub.Dx = temp_float[0].result;
        pub.Dy = temp_float[1].result;
        pub.Dz = temp_float[2].result;
        pub.Ax = temp_float[3].result;
        pub.Ay = temp_float[4].result;
        pub.Az = temp_float[5].result;
        pub.Vn = temp_float[6].result;
        pub.Vu = temp_float[7].result;
        pub.Ve = temp_float[8].result;
        pub.Altitude = temp_float[9].result;
        pub.Longitude = double(temp_int[0].result) / 10000000.0;
        pub.Lattitude = double(temp_int[1].result) / 10000000.0;
        pub.Roll = temp_float[10].result;
        pub.Heading = temp_float[11].result;
        pub.Pitch = temp_float[12].result;
        pub.Angularacceleration = temp_short.result;
        read_pub.publish(pub);
        as_pub.publish(state_as);
        //ROS_INFO_STREAM("Serial port of Solution is completed\n");
      }
    }
  }
}
