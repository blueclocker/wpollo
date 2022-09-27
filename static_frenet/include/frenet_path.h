
#ifndef _FRENET_PATH_H
#define _FRENET_PATH_H

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include"cpprobotics_types.h"

namespace cpprobotics{

class FrenetPath{
public:
  //代价函数组成部分
  float cd = 0.0;
  float cv = 0.0;
  float cf = 0.0;
  
  Vec_f t;
  //横向数据
  Vec_f d;
  Vec_f d_d;
  Vec_f d_dd;
  Vec_f d_ddd;
  //纵向数据
  Vec_f s;
  Vec_f s_d;
  Vec_f s_dd;
  Vec_f s_ddd;
//直角坐标下数据
  Vec_f x;
  Vec_f y;
  Vec_f yaw;
  Vec_f ds;
  Vec_f c;
 //用于路径检测
  float max_speed;
  float max_accel;
  float max_curvature;
  float max_accel_d;//最大减速度
  float max_left_d;  
  float max_right_d;  
};

using Vec_Path=std::vector<FrenetPath>;
}
#endif
