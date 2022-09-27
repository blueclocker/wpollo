
#ifndef _VEHICLE_DATA_H
#define  _VEHICLE_DATA_H
#define M_PI 3.14159265358979323846

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include <math.h>
#include"cpprobotics_types.h"
namespace cpprobotics{

//记录汽车处于t时刻的所有信息
class VehicleData{
    public:
 VehicleData (float ,float, float, float ,float ,float, float , float,float,float);
Vec_f Vehicle_x ();
Vec_f Vehicle_y ();
private:
   float time;
 float center_d;
 float d_v;
 float center_s;
 float s_v;
 float center_x;
 float center_y;
 float head_yaw;
 float VehWidth;
 float VehLength;
  
  float self_x00; //画出自车位置
  float self_y00;
  float self_x01;
  float self_y01;
  float self_x1;
  float self_x2;
  float self_x3;
  float self_x4;
  float self_y1;
  float self_y2;
  float self_y3;
  float self_y4;

  Vec_f x_list;
  Vec_f y_list;

};

 VehicleData::VehicleData (float t,float s, float d, float v_s,float v_d,float x, float y, float width,float length,float yaw)
 {
 time = t;
 center_d = d;
 d_v = v_d;
 center_s = s;
 s_v = v_s;
 center_x = x;
 center_y = y;
 head_yaw = yaw;
 VehWidth = width;
 VehLength = length;
self_x00 = x+ 0.5 * length * cos(yaw); //车头中间点
self_y00 = y + 0.5 * length * sin(yaw);
self_x01 = x - 0.5 * length * cos(yaw); //车尾中间点
self_y01 = y - 0.5 * length * sin(yaw);
self_x1 = self_x00 + 0.5 * width * cos(yaw + 0.5 * M_PI);
self_y1 = self_y00 + 0.5 * width * sin(yaw + 0.5 * M_PI);
self_x2 = self_x00 + 0.5 *width * cos(yaw - 0.5 * M_PI);
self_y2 = self_y00 + 0.5 * width * sin(yaw - 0.5 * M_PI);
self_x3 = self_x01 + (self_x2 - self_x00);
self_y3 = self_y01 + (self_y2 - self_y00);
self_x4 = self_x01 + (self_x1 - self_x00);
self_y4 = self_y01 + (self_y1 - self_y00);
x_list.push_back(self_x1);
x_list.push_back(self_x2);
x_list.push_back(self_x3);
x_list.push_back(self_x4);
y_list.push_back(self_y1);
y_list.push_back(self_y2);
y_list.push_back(self_y3);
y_list.push_back(self_y4);
 };
 Vec_f VehicleData::Vehicle_x (){
  return x_list;
 };
 Vec_f VehicleData::Vehicle_y (){
  return y_list;
 };
}
#endif