
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
// #include "VehicleData.h"
#include <ros/ros.h>
#include <rs_perception/PerceptionListMsg.h>
#include <nav_msgs/Odometry.h> //localization
#include <sensor_msgs/Imu.h> //acc
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>
//#include "fsd_common_msgs/Gnss.h"
#include <visualization_msgs/MarkerArray.h> //rviz visualization
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "osmmap/CarState.h"
//用于路径规划
#include "cubic_spline.h"
#include "frenet_path.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"

//用于碰撞检测
#include "collision.h"
#include "shape.h"
#include "vector2d.h"
#include "obs_ver_list.h"
#include <math.h>

#define SIM_LOOP 500
#define MAX_SPEED 50.0 / 3.6    // maximum speed [m/s]
#define MAX_ACCEL 4.0           // maximum acceleration [m/ss]
#define MAX_ACCEL_D -6.0        // maximum -acceleration [m/ss]
#define MAX_CURVATURE 0.5       // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 3.5      // maximum road width [m]
#define D_ROAD_W 0.2            // road width sampling length [m]
#define DT 0.2                  // time tick [s]
#define MAXT 5.0                // max prediction time [m]
#define MINT 4.0                // min prediction time [m]
#define TARGET_SPEED 20.0 / 3.6 // target speed [m/s]
#define D_T_S 2.5 / 3.6         // target speed sampling length [m/s]
#define N_S_SAMPLE 2            // sampling number of target speed
#define M_PI 3.14159265358979323846
#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0
#define veh_width 2.0  //自车宽度 （m）
#define veh_length 4.0 //自车长度 （m）
#define tau 2.0        //车头实距时间常数（s）
#define Pi 3.14159265

typedef message_filters::sync_policies::ApproximateTime<rs_perception::PerceptionListMsg,
                                                        visualization_msgs::Marker,
                                                        osmmap::CarState>
    syncPolicy;

using namespace cpprobotics;

ros::Publisher path_pub;
ros::Publisher box_array_pub_;
ros::Publisher path_all;
FrenetPath last_path;
int count_last_path = 0;
Vec_f frenet_c_point = {0.0,0.0};
Vec_f frenet_goal_point = {std::numeric_limits<float>::max(),std::numeric_limits<float>::max()};
//frenet到笛卡尔坐标系转化，可以生效
Vec_f frenet_to_cartesian(float s, float d, float d_d, Vec_f r_x, Vec_f r_y, Vec_f ryaw, Vec_f rcurvature, Vec_f rs)
//frenet坐标系到笛卡尔坐标系
{ //std::cout << "进入f_d坐标转化函数 " << std::endl;
  float x_ref = 0., y_ref = 0., theta_ref = 0., k_ref = 0.;
  Vec_f cartesian_point;
  float xx, yx, theta_x;
  for (int i = 0; i < rs.size() - 1; i++)
  {
    if ((rs[i] - s) * (rs[i + 1] - s) <= 0.)
    {
      x_ref = r_x[i];
      y_ref = r_y[i];
      theta_ref = ryaw[i];
      k_ref = rcurvature[i];
      break;
    }
  }
  xx = x_ref - d * sin(theta_ref);
  yx = y_ref + d * cos(theta_ref);
  theta_x = atan2(d_d, 1 - k_ref) + theta_ref;
  cartesian_point.push_back(xx);
  cartesian_point.push_back(yx);
  cartesian_point.push_back(theta_x);
  cartesian_point.push_back(theta_ref);
  // std::cout << "完成坐标转化函数 " << std::endl;
  return cartesian_point;
};

//笛卡尔坐标系到Frenet坐标系的转化,可以生效
Vec_f cartesian_to_frenet(float xx, float yx, Vec_f r_x, Vec_f r_y, Vec_f ryaw, Vec_f rs)
{ //笛卡尔坐标系到frenet坐标系
  // reference_path = ref_paths[lane_number].csp
  // reference_path_x = ref_paths[lane_number].x
  // reference_path_y = ref_paths[lane_number].y
  // reference_path_yaw = ref_paths[lane_number].yaw
  // index = len(reference_path_x) - 1
  float sr;
  float dis = std::numeric_limits<float>::max();
  float dot_pro1;
  // float dot_pro2;
  float absolute_d;
  float d;
  int index;
  for (int i = 0; i < r_x.size() - 1; i++)
  { //通过这个方法找到邻近点
    // dot_pro1 = (xx - r_x[i]) * (r_x[i + 1] - r_x[i]) + (yx - r_y[i]) * (r_y[i + 1] - r_y[i]);
    // dot_pro2 = (xx - r_x[i + 1]) * (r_x[i + 2] - r_x[i + 1]) + (yx - r_y[i + 1]) * (r_y[i + 2] - r_y[i + 1]);
    dot_pro1 = (xx - r_x[i]) * (xx - r_x[i]) + (yx - r_y[i]) * (yx - r_y[i]);
    // if (dot_pro1 * dot_pro2 <= 0)
    // {
    //   index = i + 1;
    //   break;
    // };
    if(dot_pro1<dis)
    {
      dis = dot_pro1;
      index = i;
    }
  };
  sr = rs[index];
  absolute_d = pow((pow((xx - r_x[index]), 2) + pow((yx - r_y[index]), 2)), 0.5);
  int A;
  if (((yx - r_y[index]) * cos(ryaw[index]) - (xx - r_x[index]) * sin(ryaw[index])) > 0)
    A = 1;
  else if (((yx - r_y[index]) * cos(ryaw[index]) - (xx - r_x[index]) * sin(ryaw[index])) < 0)
    A = -1;
  else
    A = 0;
  d = A * absolute_d;
  Vec_f point = {sr, d, ryaw[index]};
  return point;
}; //先找最近点，然后求出世纪距离d，近似求出垂直距离d

//求平方累积值
float sum_of_power(std::vector<float> value_list)
{
  float sum = 0;
  for (float item : value_list)
  {
    sum += item * item;
  }
  return sum;
};

//意图判断函数
int get_action(float s0, float s_goal, float other_vehicle_s)
{
  // action: 0-巡航；1-停车；2-跟车
  int Action = 0;
  //还没有获取到停车点
  if (s_goal - s0 > 30)
  { 
    if(other_vehicle_s> s0){
      if ((other_vehicle_s - s0 > 25)||(other_vehicle_s - s0<15))
        Action = 0;
      else
        Action = 2;
    }else{
      Action = 0;
    }
  }
  //获取到停车点
  else if (s_goal - s0 <= 30)
  {
    if (other_vehicle_s > s0)
    {
      if ((other_vehicle_s <= s_goal)&&(other_vehicle_s-s0>=15))
        Action = 2;
      else
        Action = 1;
    }
    if (other_vehicle_s <= s0)
      Action = 1;
  }
  else
    Action = 0;
  return Action;
};

//巡航下路径采样  longitudinal 3 for(Ti include 2)  latitude 2 for(Ti include 1)
Vec_Path calc_frenet_paths_cruising(
    float t_speed, float c_speed, float c_d, float c_d_d, float c_d_dd, float s0)
{
  Vec_Path fp_list;
  for (float di = -(0.5 * MAX_ROAD_WIDTH); di < (0.5 * MAX_ROAD_WIDTH); di += D_ROAD_W)
  {

    for (float Ti = MINT; Ti < MAXT; Ti += DT)// dif t 2 same d
    { // MAXT  5.0   MINT  4.0   DT  0.2
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      for (float t = 0; t < Ti; t += DT)//bu chang de bu chang
      {
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }
      float min_speed = t_speed;// - D_T_S * N_S_SAMPLE;
      if (min_speed <= 0)
        min_speed = 0.0;
      for (float tv = min_speed;
           tv < t_speed + D_T_S * N_S_SAMPLE;
           tv += D_T_S)
      {

        FrenetPath fp_bot = fp;
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();

        fp_bot.max_accel = std::numeric_limits<float>::min();
        fp_bot.max_accel_d = 0; //求最大减速度
        fp_bot.max_left_d = std::numeric_limits<float>::min();
        fp_bot.max_right_d = 0;
        for (float t_ : fp.t)
        {
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
          if (fp_bot.s_d.back() > fp_bot.max_speed)
          {
            fp_bot.max_speed = fp_bot.s_d.back();
          };
          if (fp_bot.s_dd.back() > fp_bot.max_accel)
          {
            fp_bot.max_accel = fp_bot.s_dd.back();
          };
          if (fp_bot.d.back() > fp_bot.max_left_d)
          {
            fp_bot.max_left_d = fp_bot.d.back();
          };
          // if ((fp_bot.max_accel_d<0)&&fabs(fp_bot.s_dd.back())>fabs(fp_bot.max_accel_d))
          // {
          //   fp_bot.max_accel_d = fp_bot.s_dd.back();
          // };
        }
        //求最大减速度
        for (int j = 0; j < fp_bot.s_dd.size(); j++)
        {
          if (fp_bot.s_dd[j] <= fp_bot.max_accel_d)
            fp_bot.max_accel_d = fp_bot.s_dd[j];
        }
        
        //求最大负偏移d
        for (int j = 0; j < fp_bot.d.size(); j++)
        {
          if (fp_bot.d[j] <= fp_bot.max_right_d)
            fp_bot.max_right_d = fp_bot.d[j];
        }
        float Jp = sum_of_power(fp.d_ddd);
        float Js = sum_of_power(fp_bot.s_ddd);
        float ds = (t_speed - fp_bot.s_d.back());

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
        fp_bot.cv = 0.5 * Js + 0.8 * Ti + 2 * ds;//KJKTKD
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

        fp_list.push_back(fp_bot);
      }
    }
  }
  std::cout << "巡航时采样路径数目:" << fp_list.size() << std::endl;
  return fp_list;
};

//没路下停车下路径采样
Vec_Path calc_frenet_paths_stopping_noway(
    float tar_speed, float c_speed, float c_acc, float c_d, float c_d_d, float c_d_dd, float s0, float frenet_s_goal, float frenet_d_goal)
{
  Vec_Path fp_list;
  for (float di = frenet_d_goal; di < frenet_d_goal + 0.01; di += 0.5 * D_ROAD_W)
  { // MAX_ROAD_WIDTH  7.0   D_ROAD_W  1.0
    //设定最小停车时间
    float mint = MINT;
    if (c_speed / (-MAX_ACCEL_D) < mint)
      mint = c_speed / (-MAX_ACCEL_D);
    //设定最大停车时间
    float maxt = MAXT;
    if ((10 / (c_speed + 0.1)) > maxt)
      maxt = (10 / (c_speed + 0.1));
    if (maxt >= 6)
      maxt = 6;
    for (float Ti = mint; Ti < maxt; Ti += DT)
    { // MAXT  5.0   MINT  4.0   DT  0.2
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      for (float t = 0; t < Ti; t += DT)
      {
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }
      for (float ts = frenet_s_goal - 4;///////////////problem
           ts < frenet_s_goal + 0.01;
           ts += 1)
      {

        FrenetPath fp_bot = fp;
        QuinticPolynomial lon_qp(s0, c_speed, c_acc, ts, 0, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
        fp_bot.max_accel_d = 0; //求最大减速度
        fp_bot.max_left_d = std::numeric_limits<float>::min();
        fp_bot.max_right_d = 0;
        for (float t_ : fp.t)
        {
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
          if (fp_bot.s_d.back() > fp_bot.max_speed)
          {
            fp_bot.max_speed = fp_bot.s_d.back();
          };
          if (fp_bot.s_dd.back() > fp_bot.max_accel)
          {
            fp_bot.max_accel = fp_bot.s_dd.back();
          };
          if (fp_bot.d.back() > fp_bot.max_left_d)
          {
            fp_bot.max_left_d = fp_bot.d.back();
          };
          // if ((fp_bot.max_accel_d<0)&&fabs(fp_bot.s_dd.back())>fabs(fp_bot.max_accel_d))
          // {
          //   fp_bot.max_accel_d = fp_bot.s_dd.back();
          // };
        }
        for (int j = 0; j < fp_bot.s_dd.size(); j++)
        {
          if (fp_bot.s_dd[j] <= fp_bot.max_accel_d)
            fp_bot.max_accel_d = fp_bot.s_dd[j];
        }
        //求最大负偏移d
        for (int j = 0; j < fp_bot.d.size(); j++)
        {
          if (fp_bot.d[j] <= fp_bot.max_right_d)
            fp_bot.max_right_d = fp_bot.d[j];
        }
        float Jp = sum_of_power(fp.d_ddd);
        float Js = sum_of_power(fp_bot.s_ddd);
        float ds = (tar_speed - fp_bot.s_d.back());

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
        fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

        fp_list.push_back(fp_bot);
      }
    }
  }
  std::cout << "没路生成停车点下采样路径数目:" << fp_list.size() << std::endl;
  return fp_list;
};
//跟车时路径采样
Vec_Path calc_frenet_paths_following(
    float c_speed, float c_acc, float c_d, float c_d_d, float c_d_dd, float s0, float followVeh_s, float followVeh_d, float followVeh_s_v, float followVeh_d_v)
{
  Vec_Path fp_list;
  for (float di = -(0.5 * MAX_ROAD_WIDTH - 0.5 * veh_width); di < 0.5 * MAX_ROAD_WIDTH - 0.5 * veh_width; di += D_ROAD_W)
  { // D_ROAD_W  1.0
    float mint = MINT;
    if (c_speed / (-MAX_ACCEL_D) < mint)
      mint = c_speed / (-MAX_ACCEL_D);
    //设定最大停车时间
    for (float Ti = mint; Ti < MAXT + 1; Ti += DT)
    { // MAXT  5.0   MINT  4.0   DT  0.2

      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      for (float t = 0; t < Ti; t += DT)
      {
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }
      //预测前车位置s_fv1
      float s_fv1 = followVeh_s + followVeh_s_v * Ti + 0.5 * 0 * std::pow(Ti, 2);
      float s_target = s_fv1 - tau * fabs(c_speed - followVeh_s_v)-10;
      float s_target_d = followVeh_s_v + 0 * Ti;
      float s_target_dd = 0;
      if (s_target > s0 + c_speed * Ti + 0.5 * MAX_ACCEL * std::pow(Ti, 2))
        s_target = s0 + c_speed * Ti + 0.5 * MAX_ACCEL * std::pow(Ti, 2);
      if (s_target_d > MAX_SPEED)
        s_target_d = 0.98 * MAX_SPEED;
      if (s_target_dd > MAX_ACCEL)
        s_target_dd = 0.98 * MAX_ACCEL;

      // FrenetPath fp_bot = fp;//?
      // QuinticPolynomial lon_qp(s0, c_speed, c_acc, s_target, s_target_d, s_target_dd, Ti);

      // fp_bot.max_speed = std::numeric_limits<float>::min();
      // fp_bot.max_accel = std::numeric_limits<float>::min();
      // fp_bot.max_accel_d = 0; //求最大减速度
      // fp_bot.max_left_d = std::numeric_limits<float>::min();
      // fp_bot.max_right_d = 0;
      // for (float t_ : fp.t)
      // {
      //   fp_bot.s.push_back(lon_qp.calc_point(t_));
      //   fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
      //   fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
      //   fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
      for (float tv = s_target_d;
           tv < s_target_d + D_T_S * N_S_SAMPLE;
           tv += D_T_S)
      {

        FrenetPath fp_bot = fp;
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
        fp_bot.max_accel_d = 0; //求最大减速度
        fp_bot.max_left_d = std::numeric_limits<float>::min();
        fp_bot.max_right_d = 0;
        for (float t_ : fp.t)
        {
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));

        if (fp_bot.s_d.back() > fp_bot.max_speed)
        {
          fp_bot.max_speed = fp_bot.s_d.back();
        };
        if (fp_bot.s_dd.back() > fp_bot.max_accel)
        {
          fp_bot.max_accel = fp_bot.s_dd.back();
        };
        if (fp_bot.d.back() > fp_bot.max_left_d)
        {
          fp_bot.max_left_d = fp_bot.d.back();
        };
      }
      for (int j = 0; j < fp_bot.s_dd.size(); j++)
      {
        if (fp_bot.s_dd[j] <= fp_bot.max_accel_d)
          fp_bot.max_accel_d = fp_bot.s_dd[j];
      }
      //               //求最大负偏移d
      for (int j = 0; j < fp_bot.d.size(); j++)
      {
        if (fp_bot.d[j] <= fp_bot.max_right_d)
          fp_bot.max_right_d = fp_bot.d[j];
      }

      float Jp = sum_of_power(fp.d_ddd);
      float Js = sum_of_power(fp_bot.s_ddd);
      float ds = (s_target_d - fp_bot.s_d.back());

      fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back() - followVeh_d, 2);
      fp_bot.cv = KJ * Js + KT * Ti+10*ds;
      fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

      fp_list.push_back(fp_bot);
    }
  }
  }
  std::cout << "跟车时采样路径数目:" << fp_list.size() << std::endl;
  return fp_list;
}

//停车下的路径采样,第二版
Vec_Path calc_frenet_paths_stopping(
    float c_speed, float c_acc, float c_d, float c_d_d, float c_d_dd, float s0, float frenet_s_goal, float frenet_d_goal)
{
  Vec_Path fp_list;
  float s_goal = frenet_s_goal;
  float d_goal = frenet_d_goal;
  //设定最小停车时间
  float mint = MINT;
  if (c_speed / (-MAX_ACCEL_D) < mint)
    mint = c_speed / (-MAX_ACCEL_D);
  //设定最大停车时间
  float maxt = MAXT;
  if ((2 * (s_goal - s0) / (c_speed + 0.1)) > maxt)
    maxt = (2 * (s_goal - s0) / (c_speed + 0.1));
  if (maxt >= 6)
    maxt = 6;
  for (float Ti = mint; Ti < maxt; Ti += DT)
  { // MAXT  5.0   MINT  4.0   DT  0.2
    FrenetPath fp;
    //  if (( s0 + c_speed* Ti + 0.5*MAX_ACCEL_D*std::pow(Ti,2)) > s_goal) //设定最近停车点,有问题
    //    {
    //      s_goal = s0 + 0.5* MAX_ACCEL_D*std::pow(Ti,2);
    //      std::cout << "重设T = " << Ti << " 时停车点 s_goal = " << s_goal<< std::endl;
    //    }

    //  else   s_goal = frenet_s_goal;
    QuinticPolynomial lon_qp(s0, c_speed, c_acc, s_goal, 0.0, 0.0, Ti);
    for (float t = 0; t < Ti; t += DT)
    {
      fp.t.push_back(t);
      fp.s.push_back(lon_qp.calc_point(t));
      fp.s_d.push_back(lon_qp.calc_first_derivative(t));
      fp.s_dd.push_back(lon_qp.calc_second_derivative(t));
      fp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
    }

    FrenetPath fp_bot = fp;
    QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, d_goal, 0.0, 0.0, Ti);
    fp_bot.max_speed = std::numeric_limits<float>::min();
    fp_bot.max_accel = std::numeric_limits<float>::min();
    // fp_bot.max_accel_d =fabs(-std::numeric_limits<float>::min());  //求最大减速度的有问题
    fp_bot.max_accel_d = 0;
    fp_bot.max_left_d = std::numeric_limits<float>::min();
    fp_bot.max_right_d = 0;
    for (float t_ : fp.t)
    {
      fp_bot.d.push_back(lat_qp.calc_point(t_));
      fp_bot.d_d.push_back(lat_qp.calc_first_derivative(t_));
      fp_bot.d_dd.push_back(lat_qp.calc_second_derivative(t_));
      fp_bot.d_ddd.push_back(lat_qp.calc_third_derivative(t_));
      if (fp_bot.s_d.back() > fp_bot.max_speed)
      {
        fp_bot.max_speed = fp_bot.s_d.back();
      };
      if (fp_bot.s_dd.back() > fp_bot.max_accel)
      {
        fp_bot.max_accel = fp_bot.s_dd.back();
      };
      if (fp_bot.d.back() > fp_bot.max_left_d)
      {
        fp_bot.max_left_d = fp_bot.d.back();
      };
      // if ((fp_bot.max_accel_d<1e-7)&&fabs(fp_bot.s_dd.back())>=fabs(fp_bot.max_accel_d))
      //   //  if (fp_bot.s_dd.back()<=fp_bot.max_accel_d)
      // {
      //   fp_bot.max_accel_d = fp_bot.s_dd.back();
      // };
      for (int j = 0; j < fp_bot.s_dd.size(); j++)
      {
        if (fp_bot.s_dd[j] <= fp_bot.max_accel_d)
          fp_bot.max_accel_d = fp_bot.s_dd[j];
      }
      //求最大负偏移d
      for (int j = 0; j < fp_bot.d.size(); j++)
      {
        if (fp_bot.d[j] <= fp_bot.max_right_d)
          fp_bot.max_right_d = fp_bot.d[j];
      }
    }
    float Jp = sum_of_power(fp.d_ddd);
    float Js = sum_of_power(fp_bot.s_ddd);
    float ds = s_goal - frenet_s_goal;
    // fp_bot.cd = KJ * Jp + KT * Ti ;
    // fp_bot.cv = KJ * Js + KT * Ti +10 * KD * ds;

    fp_bot.cd = -KT * Ti;
    fp_bot.cv = -1 * Ti + 1.0 * fp_bot.max_accel;//KT
    fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;
    fp_list.push_back(fp_bot);
  }
  // std::cout << "停车时采样路径数目:" << fp_list.size() << std::endl;
  return fp_list;
};

//计算直角坐标系信息
void calc_global_paths(Vec_Path &path_list, Spline2D csp)
{ 
  for (Vec_Path::iterator path_p = path_list.begin(); path_p != path_list.end(); path_p++)
  { 
    for (unsigned int i = 0; i < path_p->s.size(); i++)
    { 
      if (path_p->s[i] >= csp.s.back())
      { 
        break;
      }
      std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);
      float iyaw = csp.calc_yaw(path_p->s[i]);
      float di = path_p->d[i];
      float x = poi[0] + di * std::cos(iyaw + M_PI / 2.0);
      float y = poi[1] + di * std::sin(iyaw + M_PI / 2.0);
      path_p->x.push_back(x);
      path_p->y.push_back(y);
    }
    if(path_p->x.size()>1)///////////
    { 
      for (int i = 0; i < path_p->x.size() - 1; i++)//////path_p->x.size()=1?
      { 
        float dx = path_p->x[i + 1] - path_p->x[i];
        float dy = path_p->y[i + 1] - path_p->y[i];
        float yaw = std::atan2(dy, dx);
        if(yaw<0)
        {
          yaw += 2*Pi;
        }

  //    Eigen::Quaterniond R = {car_data->carPose.orientation.w, car_data->carPose.orientation.x,car_data->carPose.orientation.y,car_data->carPose.orientation.z};
  //    Eigen::Vector3d t = {car_data->carPose.position.x, car_data->carPose.position.y,car_data->carPose.position.z}; 
  //    Eigen::Vector3d P = {obs_data->perceptions[per_msgs].polygon_point[obs_point].x,obs_data->perceptions[per_msgs].polygon_point[obs_point].y,obs_data->perceptions[per_msgs].polygon_point[obs_point].z};
  //    Eigen::Vector3d tmp_P = R*P+t;

        path_p->yaw.push_back(yaw); //返回弧度值
        path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
      }
      path_p->yaw.push_back(path_p->yaw.back());//?
      path_p->ds.push_back(path_p->ds.back());
      path_p->max_curvature = std::numeric_limits<float>::min();
      int maxid = 0;
      for (int i = 0; i < path_p->x.size() - 1; i++)
      { 
        path_p->c.push_back((path_p->yaw[i + 1] - path_p->yaw[i]) / path_p->ds[i]);
        if (path_p->c.back() > path_p->max_curvature)
        {
          path_p->max_curvature = path_p->c.back();
          maxid = i;
        }
      }
    // std::cout << "path_p->max_curvature"<<path_p->max_curvature<<"maxc_x_y"<<path_p->x[maxid]<<"  "<<path_p->y[maxid]<<std::endl;
    }
    else{
      for (int i = 0; i < path_p->x.size(); i++){
        path_p->yaw.push_back(last_path.yaw.back());
        path_p->ds.push_back(0);
        path_p->max_curvature = 0;
      }
      std::cout << "wu lu jing dian"<<std::endl;
    }
  }
};

//碰撞检测
bool check_collision(FrenetPath path, const Polygon_list obs_list, Vec_f velocity_list)
{
  if (!obs_list.empty())
  {
    int i;
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
    int num_veh_self = 4;
    Vector2D *vector_veh_self = new Vector2D[num_veh_self];
    cv::Point veh_points[1][20];
    cv::Point2i veh_points_plt[1][20];

    for (i = 0; i < path.x.size(); i++)
    {
      for (int j =0; j <obs_list.size(); j++){
        // std::cout << "x_"<< path.x[i] + 0.5 * veh_length * cos(path.yaw[i])
        //                  <<" "<< obs_list[j].vertices->x
        //                  <<" "<< path.x[i] + 0.5 * veh_length * cos(path.yaw[i])-path.t[i]*velocity_list[j]
        //                  <<" "<< obs_list[j].vertices->x+path.t[i]*velocity_list[j]
        //                  <<std::endl;
        self_x00 = path.x[i] + 0.5 * veh_length * cos(path.yaw[i])-path.t[i]*velocity_list[j]; //车头中间点
        self_y00 = path.y[i] + 0.5 * veh_length * sin(path.yaw[i]);
        self_x01 = path.x[i] - 0.5 * veh_length * cos(path.yaw[i])-path.t[i]*velocity_list[j]; //车尾中间点
        self_y01 = path.y[i] - 0.5 * veh_length * sin(path.yaw[i]);
        self_x1 = self_x00 + 0.5 * veh_width * cos(path.yaw[i] + 0.5 * M_PI);
        self_y1 = self_y00 + 0.5 * veh_width * sin(path.yaw[i] + 0.5 * M_PI);
        self_x2 = self_x00 + 0.5 * veh_width * cos(path.yaw[i] - 0.5 * M_PI);
        self_y2 = self_y00 + 0.5 * veh_width * sin(path.yaw[i] - 0.5 * M_PI);
        self_x3 = self_x01 + (self_x2 - self_x00);
        self_y3 = self_y01 + (self_y2 - self_y00);
        self_x4 = self_x01 + (self_x1 - self_x00);
        self_y4 = self_y01 + (self_y1 - self_y00);

        vector_veh_self[0] = Vector2D(self_x1, self_y1); //用于碰撞检测
        vector_veh_self[1] = Vector2D(self_x2, self_y2);
        vector_veh_self[2] = Vector2D(self_x3, self_y3);
        vector_veh_self[3] = Vector2D(self_x4, self_y4);

        Polygon polygon_veh = Polygon(num_veh_self, vector_veh_self); //碰撞检测多边体
        veh_points[0][0] = cv::Point2f(self_x1, self_y1);
        veh_points[0][1] = cv::Point2f(self_x2, self_y2);
        veh_points[0][2] = cv::Point2f(self_x3, self_y3);
        veh_points[0][3] = cv::Point2f(self_x4, self_y4);
      
        Simplex simplex;
        if (intersect(polygon_veh, obs_list[j]))
        {
          return true;
        }
          
        else{
          continue;
        }
        // else{
        //   // Vec_f frenet_obs_predication = cartesian_to_frenet(obs.vertices->x,obs.vertices->y,r_x,r_y,ryaw,rs);
        //   // float s = frenet_obs_predication[0] + path.t[i]*20/3.6;
        //   // float d = frenet_obs_predication[1];
        //   // Vec_f cartesian_obs_predication = frenet_to_cartesian(s,d,0.0,r_x,r_y,ryaw,rcurvature,rs);
        //   // dyn_obs.vertices->x = cartesian_obs_predication[0];
        //   // dyn_obs.vertices->y = cartesian_obs_predication[1];
        //   Polygon temp_obs = obs;
        //   temp_obs.vertices->x = obs.vertices->x +path.t[i]*obs.vertices->velocity_x;
        //   temp_obs.vertices->y = obs.vertices->y +path.t[i]*obs.vertices->velocity_y;          
        //   Simplex simplex;
        //   if (intersect(polygon_veh, temp_obs)){
        //     return true;
        //   }
          
        //   else
        //     continue;
        // }
      }
    }
  }
  return false;
};

//路径筛选
Vec_Path check_paths(Vec_Path path_list, const Polygon_list obs_list,Vec_f velocity_list)
{
  std::cout << "进入路径筛选" << std::endl;
  // std::cout << "path_list"<<path_list.size() << std::endl;
  Vec_Path output_fp_list;
  for (FrenetPath &path : path_list)//speed      acc_D!
  { 
    if(path.s.size()>0){
      if(!check_collision(path, obs_list,velocity_list))
      { 
        if (  path.max_left_d < 1.75 &&path.max_right_d > -1.75 )
        { 
          if(path.max_speed < MAX_SPEED)
          { 
            if(path.max_curvature < MAX_CURVATURE){
              if(path.max_accel < MAX_ACCEL ){
                if(path.max_accel_d > MAX_ACCEL_D)
                output_fp_list.push_back(path);
              }
            }
          }
        }
      }
    }  
  }
  std::cout << "有可用路径 " << output_fp_list.size() << " 条。" << std::endl;
  return output_fp_list;
};

Vec_Path check_paths_stop(Vec_Path path_list, const Polygon_list obs_list,Vec_f velocity_list,float yaw_goal)
{
  std::cout << "进入路径筛选" << std::endl;
  Vec_Path output_fp_list;
  // if(path_list.size()>0){
    for (FrenetPath &path : path_list)//speed      acc_D!
    { 
      if(path.s.size()>0)
      {
        if(!check_collision(path, obs_list,velocity_list))
        { 
          if (  (path.max_left_d < 1.75) &&(path.max_right_d > -1.75) )
          { 
            if(path.max_speed < MAX_SPEED)
            { 
              if(path.max_curvature < MAX_CURVATURE)
              {
                if(path.max_accel < 0.25*MAX_ACCEL )
                { 
                  if(std::abs(std::abs(path.yaw.back())-std::abs(yaw_goal))<0.1)
                  {
                    if(path.max_accel_d > MAX_ACCEL_D)
                    output_fp_list.push_back(path);
                  }
                }
              }
            }
          }
        }
      }
    }
  std::cout << "有可用路径 " << output_fp_list.size() << " 条。" << std::endl;
  return output_fp_list;
};

FrenetPath frenet_optimal_planning_cruising( //巡航下路径规划
    float T_speed, Spline2D csp, float s0, float c_speed,
    float c_d, float c_d_d, float c_d_dd, Polygon_list obs_list,Vec_f velocity_list)
{
  Vec_Path fp_list = calc_frenet_paths_cruising(T_speed, c_speed, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fp_list, csp);
  Vec_Path save_paths = check_paths(fp_list, obs_list,velocity_list);
  if (save_paths.empty())
    std::cout << "没有采样出最优路径" << std::endl;
  FrenetPath final_path;
  float min_cost = std::numeric_limits<float>::max();
  for (auto path : save_paths)
  {
    if (min_cost >= path.cf)
    {
      min_cost = path.cf;
      final_path = path;
    }
  }
  std::cout << "输出巡航时最优路径" << std::endl;
  return final_path;
};

FrenetPath frenet_optimal_planning_stopping( //停车下路径规划
    float T_speed, Spline2D csp, float s0, float c_speed, float c_acc,
    float c_d, float c_d_d, float c_d_dd, float s_goal, float d_goal, float yaw_goal, 
    Polygon_list obs_list, FrenetPath last_path,Vec_f velocity_list)
{ 
  std::cout << "stop_point"<<s_goal<<" "<<d_goal<<std::endl;
  Vec_Path fp_list = calc_frenet_paths_stopping(c_speed, c_acc, c_d, c_d_d, c_d_dd, s0, s_goal, d_goal);
  std::cout << "完成第一部采样" <<std::endl;
  std::cout <<fp_list.size() << std::endl;
  calc_global_paths(fp_list, csp);
  std::cout << "完成路径直角坐标系信息补充" <<std::endl;
  Vec_Path save_paths = check_paths_stop(fp_list, obs_list,velocity_list,yaw_goal);
  std::cout << "完成碰撞检测" <<std::endl;
  FrenetPath final_path;
  float min_cost = std::numeric_limits<float>::max();
  for (auto path : save_paths)
  {
    if (min_cost >= path.cf)
    {
      min_cost = path.cf;
      final_path = path;
    }
  }
  // if (final_path.s.empty()  )
  // final_path = last_path;
  std::cout << "输出停车时最优路径" << std::endl;
  return final_path;
};

//没路时自己设停车点,测试修改版
FrenetPath frenet_optimal_planning_stopping_noway(
    float T_speed, Spline2D csp, float s0, float c_speed, float c_acc,
    float c_d, float c_d_d, float c_d_dd, float s_goal, float d_goal, 
    Polygon_list obs_list, FrenetPath last_path,Vec_f velocity_list)
{
  std::cout << "进入没路停车时采样第一层函数" << std::endl;
  Vec_Path fp_list = calc_frenet_paths_stopping_noway(T_speed, c_speed, c_acc, c_d, c_d_d, c_d_dd, s0, s_goal, d_goal);
  std::cout << "fp_list"<<fp_list.size() << std::endl;
  // std::cout << "完成第一部采样" <<std::endl;
  calc_global_paths(fp_list, csp);
  ROS_INFO("GLOBAL PATH NOWAY");
  Vec_Path save_paths = check_paths(fp_list, obs_list,velocity_list);
  std::cout << "SAVE PATH NOWAY" <<save_paths.size()<< std::endl;
  FrenetPath final_path;
  float min_cost = std::numeric_limits<float>::max();
  for (auto path : save_paths)
  {
    if (min_cost >= path.cf)
    {
      min_cost = path.cf;
      final_path = path;
    }
    std::cout << "输出自定义停车时最优路径" << std::endl;
    return final_path;
  }

  // if (final_path.s.empty()  )
  // final_path = last_path;
  std::cout << "输出没路时停车时最优路径" << std::endl;
  return final_path;
};

//跟车下路径规划
FrenetPath frenet_optimal_planning_following(
    float T_speed, Spline2D csp, float s0, float c_speed, float c_acc,
    float c_d, float c_d_d, float c_d_dd, float ThefollowVeh_s, float ThefollowVeh_d, 
    float ThefollowVeh_s_v, float ThefollowVeh_d_v, Polygon_list obs_list, FrenetPath last_path,Vec_f velocity_list)
{ //std::cout << "进入到停车时采样第一层函数" <<std::endl;
  Vec_Path fp_list = calc_frenet_paths_following(c_speed, c_acc, c_d, c_d_d, c_d_dd, s0, ThefollowVeh_s, ThefollowVeh_d, ThefollowVeh_s_v, ThefollowVeh_d_v);
  // std::cout << "完成第一部采样" <<std::endl;
  calc_global_paths(fp_list, csp);
  // std::cout << "完成路径直角坐标系信息补充" <<std::endl;
  Vec_Path save_paths = check_paths(fp_list, obs_list,velocity_list);
  // std::cout << "完成碰撞检测" <<std::endl;
  FrenetPath final_path;
  float min_cost = std::numeric_limits<float>::max();
  for (auto path : save_paths)
  {
    if (min_cost >= path.cf)
    {
      min_cost = path.cf;
      final_path = path;
    }
  }
  // if (final_path.s.empty()  )
  // final_path = last_path;
  std::cout << "输出跟车时最优路径" << std::endl;
  return final_path;
};

//行车位置预测
Vec_f LocationPrediction(float now_s, float now_d, float s_speed, float d_speed)
{
  Vec_f NextLocation;
  float next_s = now_s + s_speed * DT;
  float next_d = now_d + d_speed * DT;
  NextLocation.push_back(next_s);
  NextLocation.push_back(next_d);
  return NextLocation;
};

FrenetPath path_erase_first_point(FrenetPath &path, Vec_f frenet_c_point)
{
  path.s.erase(path.s.begin() + 0);
  path.s_d.erase(path.s_d.begin() + 0);
  path.s_dd.erase(path.s_dd.begin() + 0);
  path.s_ddd.erase(path.s_ddd.begin() + 0);
  path.t.erase(path.t.begin() + last_path.t.size() - 1);//t earse last one;
  path.d.erase(path.d.begin() + 0);
  path.d_d.erase(path.d_d.begin() + 0);
  path.d_dd.erase(path.d_dd.begin() + 0);
  path.d_ddd.erase(path.d_ddd.begin() + 0);
  path.x.erase(path.x.begin() + 0);
  path.y.erase(path.y.begin() + 0);
  path.yaw.erase(path.yaw.begin() + 0);
  path.ds.erase(path.ds.begin() + 0);
  path.c.erase(path.c.begin() + 0);
  for(int i = 0; i < path.s.size(); ++i)
  {
    path.s[i]-=frenet_c_point[0];///need to use transformation of coordinates function///
    path.d[i]-=frenet_c_point[1];
  }
  return path;
}


void callback(const rs_perception::PerceptionListMsg::ConstPtr &obs_data,
              const visualization_msgs::Marker::ConstPtr &path_data,
              const osmmap::CarState::ConstPtr &car_data)
{
  time_t begin = clock();
  //定义目标速度
  float target_speed = TARGET_SPEED;

  float c_speed_x = car_data->velocity.x;
  float c_speed_y = car_data->velocity.y;
  float c_speed = std::sqrt(c_speed_x * c_speed_x + c_speed_y * c_speed_y);
  float c_x = car_data->carPose.position.x;
  float c_y = car_data->carPose.position.y;
  float c_acc = 0; //imu_data
  float c_d_d = 0.0;
  float c_d_dd = 0.0;
  float x_goal_give = path_data->points.back().x; //停车目标点,get from map
  float y_goal_give = path_data->points.back().y;

  Vec_f wx, wy;
  for (size_t j = 0; j < path_data->points.size(); ++j)
  {
    wx.push_back(path_data->points[j].x);
    wy.push_back(path_data->points[j].y);
  }

  Spline2D csp_obj(wx, wy);
  Vec_f r_x;
  Vec_f r_y;
  Vec_f ryaw;
  Vec_f rcurvature;
  Vec_f rs;
  //补充参考线信息
  for (float i = 0; i < csp_obj.s.back(); i += 0.1)
  {
    std::array<float, 2> point_ = csp_obj.calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    ryaw.push_back(csp_obj.calc_yaw(i));
    rcurvature.push_back(csp_obj.calc_curvature(i));
    rs.push_back(i);
  }
  float max_curvature = 0.0;
  for (int i = 0; i < rcurvature.size(); i++)
  { 
    if (abs(rcurvature[i]) > max_curvature)
    {
      max_curvature = rcurvature[i];
    }
  }
  //初始信息
  frenet_c_point = cartesian_to_frenet(c_x, c_y, r_x, r_y, ryaw, rs);
  std::cout << "s0 = " << frenet_c_point[0] << " , d0 = " << frenet_c_point[1] << " , x0 = " <<  c_x <<  " , y0 = " <<  c_y << std::endl;
  float c_d = frenet_c_point[1];
  float s0 = frenet_c_point[0];
  frenet_goal_point = cartesian_to_frenet(x_goal_give, y_goal_give, r_x, r_y, ryaw, rs); //求终点直角坐标系信息
  std::cout << "s = " << frenet_goal_point[0] << " , d = " << frenet_goal_point[1] << " , x = " <<  x_goal_give <<  " , y = " <<  y_goal_give << std::endl;

  ListObsVer Obstacle_Vertexes_list;
  //obs_ car to map
  Eigen::Quaterniond R = {car_data->carPose.orientation.w, car_data->carPose.orientation.x,car_data->carPose.orientation.y,car_data->carPose.orientation.z};
  Eigen::Vector3d t = {car_data->carPose.position.x, car_data->carPose.position.y,car_data->carPose.position.z}; 

  Simplex simplex;
  // 这个是演示程序输出障碍物包围盒的列表
  std::vector<std::vector<double>> obstcles_list_vertex;  

  //此列表里存放着class ObstacleVertexes（obs_ver_list.h里定义的），每一个ObstacleVertexes存放着端点信息
  for(size_t per_msgs= 0; per_msgs <obs_data->perceptions.size(); ++per_msgs)
  {
    ObstacleVertexes *OBSvertexes = new ObstacleVertexes;
    for(size_t obs_point = 0; obs_point<obs_data->perceptions[per_msgs].polygon_point.size();++obs_point)
    {
      Eigen::Vector3d P = {obs_data->perceptions[per_msgs].polygon_point[obs_point].x,obs_data->perceptions[per_msgs].polygon_point[obs_point].y,obs_data->perceptions[per_msgs].polygon_point[obs_point].z};
      Eigen::Vector3d tmp_P = R*P+t;
      float x = tmp_P.x();
      float y = tmp_P.y();
      OBSvertexes->vertexes_list.push_back({x, y});
    }
    Obstacle_Vertexes_list.push_back(*OBSvertexes);
  }
  Vec_f velocity_list;
  //障碍物端点数目列表
  std::vector<int> num_vertices_obstcle;
  std::vector<Vector2D*> vector_obstcle;
  //循环开始，对于障碍物列表里每一个障碍物端点信息
  for(int j=0;j<Obstacle_Vertexes_list.size();j++)
  {
    num_vertices_obstcle.push_back(Obstacle_Vertexes_list[j].vertexes_list.size());
    Vector2D *vector_obstcle_data = new Vector2D[num_vertices_obstcle[j]];
    for (int z=0;z<num_vertices_obstcle[j];z++)
    {//在避障信息列表里存入点信息
      vector_obstcle_data[z] = Vector2D(Obstacle_Vertexes_list[j].vertexes_list[z][0],Obstacle_Vertexes_list[j].vertexes_list[z][1]);
    }
    //碰撞障碍物列表里存放障碍物
    vector_obstcle.push_back(vector_obstcle_data);
    velocity_list.push_back(0);
  }

  //意图代号，0-巡航，1-停车，2-跟车
  int action = 0;

  FrenetPath final_path;
  FrenetPath alter_final_path;

  int stop_and_restart = 0;
  float otherVeh_1_s = 0;
  //障碍物储存列表
  Polygon_list obstcles_list;

  //障碍物列表更新
  std::cout << "进入列表更新 " << std::endl;
  for (int k = 0; k < num_vertices_obstcle.size(); k++)
  { 
    Polygon temp_polygon(num_vertices_obstcle.at(k), vector_obstcle.at(k));
    obstcles_list.push_back(temp_polygon); //用于碰撞检测
  }
  std::cout<<"obstcles_list.size"<<obstcles_list.size()<<std::endl;
  //简化意图获取
  action = get_action(s0, frenet_goal_point[0], otherVeh_1_s);

  if (action == 0)
  {
    final_path = frenet_optimal_planning_cruising(target_speed, csp_obj, s0, c_speed, c_d, c_d_d,
                                                  c_d_dd, obstcles_list,velocity_list);
  }
  else if (action == 1&&(frenet_goal_point[0]>frenet_c_point[0]))
  {
    final_path = frenet_optimal_planning_stopping(target_speed, csp_obj, s0, c_speed, c_acc, c_d, c_d_d,
                                                  c_d_dd, frenet_goal_point[0], frenet_goal_point[1], frenet_goal_point[2], obstcles_list,last_path,velocity_list);
  }
  // else if (action == 2){
  //   final_path = frenet_optimal_planning_following(target_speed,csp_obj, s0, c_speed,c_acc, c_d, c_d_d,
  //                                                c_d_dd, otherVeh_1_s,otherVeh_1_d,otherVeh_1_v_s,otherVeh_1_v_d,obstcles_list,last_path,velocity_list);
  //   std::cout << "完成跟车路径采样和信息补充" <<std::endl;

    // Vec_Path fp_list = calc_frenet_paths_following(c_speed, c_acc, c_d, c_d_d, c_d_dd, s0, otherVeh_1_s,otherVeh_1_d,otherVeh_1_v_s,otherVeh_1_v_d);
    // calc_global_paths(fp_list, csp_obj);
    // Vec_Path save_paths;
    // for(auto path : fp_list){
    //   if(!check_collision(path,obstcles_list,velocity_list))
    //   save_paths.push_back(path);
    // }
    // visualization_msgs::MarkerArray all_path_array;
    // for(int i = 0; i < save_paths.size(); ++i)
    // {
    //   visualization_msgs::Marker marker;
    //   marker.lifetime = ros::Duration(0.1);
    //   marker.header = path_data->header;
    //   marker.type = visualization_msgs::Marker::LINE_STRIP;
    //   marker.action = visualization_msgs::Marker::ADD;
    //   marker.id = i;
    //   marker.ns = "marker";
    //   marker.scale.x = 0.1;
    //   marker.pose.orientation.x = 0.0;
    //   marker.pose.orientation.y = 0.0;
    //   marker.pose.orientation.z = 0.0;
    //   marker.pose.orientation.w = 0.5;
    //   marker.color.a = 1.0;
    //   marker.color.g = 0.0;
    //   marker.color.b = 0.0;
    //   marker.color.r = 1.0;

    //   for (size_t j = 0; j < save_paths[i].x.size(); ++j)
    //   { 
    //     geometry_msgs::Point p;
    //     p.x = save_paths[i].x[j];
    //     p.y = save_paths[i].y[j];
    //     p.z = 0;
    //     marker.points.push_back(p);
    //   }
    //   all_path_array.markers.push_back(marker);
    // }
    // path_all.publish(all_path_array);

    // Vec_Path save_paths = check_paths(fp_list, obstcles_list);
    // if (save_paths.empty())
    // std::cout << "没有采样出最优路径" << std::endl;
    // float min_cost = std::numeric_limits<float>::max();
    // for (auto path : save_paths)
    // {
    //   if (min_cost >= path.cf)
    //   {
    //     min_cost = path.cf;
    //     final_path = path;
    //   }
    // }
  // };

  //如果没有规划出来路径，判断是否调用上次最优路径/////////use last_path cos some problem
  if (final_path.s.empty() && last_path.s.size() > 2 && (count_last_path < 5)&&(frenet_goal_point[0]>frenet_c_point[0]))
  {
    std::cout << "准备调用上次路径" << std::endl;

    // float target_speed_noway = 10/3.6;  //没有路的时候目标速度给低一点
    // target_speed = TARGET_SPEED/(1+100*final_path.max_curvature);
    target_speed = TARGET_SPEED;
    last_path = path_erase_first_point(last_path , frenet_c_point);////stop has problem!!!!!   s,d value not change
    count_last_path += 1;
    std::cout << "第 " << count_last_path << " 次调用上一次最优路径 。" << std::endl;
    if (!check_collision(last_path, obstcles_list,velocity_list))
      final_path = last_path;
    else
    {
      // frenet_goal_point = cartesian_to_frenet(last_path.x.back(),last_path.y.back(),r_x,r_y,ryaw,rs);
      alter_final_path = frenet_optimal_planning_stopping(target_speed, csp_obj, s0, c_speed, c_acc, c_d, c_d_d,
                                                          c_d_dd, last_path.s.back(), last_path.d.back(), last_path.yaw.back(), obstcles_list, last_path,velocity_list);////////////////////////////////////////////////
      if (!alter_final_path.s.empty())
      {
        std::cout << "有暂时停车路径" << std::endl;
        final_path = alter_final_path;
        if (std::pow(s0 - final_path.s.back(), 2) < 0.2)
        {
          std::cout << "到达设定点2" << std::endl;
        }
      }
      else
      {
        std::cout << "没找到其他可行路!!建议停车!!stop!stop!stop!--1" << std::endl;
      }
    }
  }
  else if (final_path.s.empty() && last_path.s.size() > 2 && (count_last_path >= 5))
  { 
    std::cout << "无法调用上次路径,准备直接生成停车路径" << std::endl;
    // last_path = path_erase_first_point(last_path);
    // s0 = last_path.s[0];
    // c_d = last_path.d[0];
    // c_d_d = last_path.d_d[0];
    // c_d_dd = last_path.d_dd[0];
    // c_speed = last_path.s_d[0];
    // c_acc = last_path.s_dd[0];
    frenet_goal_point = cartesian_to_frenet(last_path.x.back(),last_path.y.back(),r_x,r_y,ryaw,rs);
    alter_final_path = frenet_optimal_planning_stopping(target_speed, csp_obj, s0, c_speed, c_acc, c_d, c_d_d,
                                                        c_d_dd, frenet_goal_point[0], frenet_goal_point[1], frenet_goal_point[2], obstcles_list,last_path,velocity_list);
    if (!alter_final_path.s.empty())
    {
      std::cout << "有暂时停车路径" << std::endl;
      final_path = alter_final_path;
      frenet_goal_point[0] = final_path.s.back();
      frenet_goal_point[1] = final_path.d.back();
    }
    else
    {
      std::cout << "没找到其他可行路!!建议停车!!stop!stop!stop!--2" << std::endl;
      ////// to ying fu w , code below neet to change//////
      // float min = 0,index = 0;
      // for(int i = 0; i <center_x.size(); ++i)
      // {
      //   float temp=std::abs(c_x-center_x[i]);
      //   if(temp<min)
      //   {
      //     index = i;
      //     min = temp;
      //   }
      // }
      // frenet_goal_point = cartesian_to_frenet(center_x[index]-3,center_y[index],r_x, r_y, ryaw, rs);
      // alter_final_path = frenet_optimal_planning_stopping(target_speed, csp_obj, s0, c_speed, c_acc, c_d, c_d_d, c_d_dd, frenet_goal_point[0], frenet_goal_point[1], obstcles_list, last_path);
      // if (!alter_final_path.s.empty())
      // {
      //   std::cout << "有暂时停车路径" << std::endl;
      //   final_path = alter_final_path;
      // }
    }
  }
  else if (final_path.s.empty() && last_path.s.size() <= 2)
  {
    std::cout << "都不行,没找到其他可行路!!建议停车!!stop!stop!stop!--3" << std::endl;
  }
  else
  {
    std::cout << "规划成功" << std::endl;
    count_last_path = 0;
  }

  if (final_path.s.size() <= 2)
  {
    stop_and_restart = 1;
  };
  std::cout << "state = "<<stop_and_restart<<std::endl;
  if(final_path.s.size()>1)
  last_path = final_path;
  else if(last_path.s.size()>1) 
  last_path = path_erase_first_point(last_path,frenet_c_point);

  visualization_msgs::MarkerArray path_array;
// visualization_msgs::MarkerArray all_path_array;

  if(final_path.s.size()>0)
  {
    visualization_msgs::Marker marker;
    marker.lifetime = ros::Duration(0.1);
    marker.header = path_data->header;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.ns = "marker";
    marker.scale.x = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.r = 0.0;

    for (size_t j = 0; j < final_path.x.size(); ++j)
    { 
      geometry_msgs::Point p;
      p.x = final_path.x[j];
      p.y = final_path.y[j];
      p.z = 0;
      marker.points.push_back(p);
    }

    path_array.markers.clear();
    path_array.markers.push_back(marker);
    path_pub.publish(path_array);
  }

  //清碰撞检测列表
  obstcles_list.clear();
  time_t finish = clock();
  std::cout << "time =  "<<(double)(finish-begin)/CLOCKS_PER_SEC <<std::endl;
}

// void callback(const rs_perception::PerceptionListMsg::ConstPtr &obs_data,
//               const visualization_msgs::Marker::ConstPtr &path_data,
//               const osmmap::carState::ConstPtr &car_data)
// {
//   std::cout << "时间同步后毫米波雷达时间戳是:  " << path_data->header.stamp<< std::endl;
//   std::cout << "时间同步后激光雷达时间戳是:    " << car_data->header.stamp << std::endl;
// }
////主程序开始
int main(int argc, char **argv)
{
  ros::init(argc, argv, "frenet_trajectory");
  ros::NodeHandle nh;

  message_filters::Subscriber<rs_perception::PerceptionListMsg> obs_sub(nh, "/adaptive_clustering/rs_percept_result", 1);
  message_filters::Subscriber<visualization_msgs::Marker> r_path_sub(nh, "/mapio/golbalpath_info", 1);
  message_filters::Subscriber<osmmap::CarState>carstate_sub(nh, "/mapio/carstate_info", 1);
  //message_filters::Subscriber<nav_msgs::Odometry> gps_sub(nh, "gnss_odometry", 1);
  //message_filters::Subscriber<sensor_msgs::Imu> acc_sub(nh, "gnss_imu", 1);
  // message_filters::Subscriber<fsd_common_msgs::Gnss> yaw_sub(nh, "gnss_odom", 1);
  path_all = nh.advertise<visualization_msgs::MarkerArray>("all_path", 1);
  path_pub = nh.advertise<visualization_msgs::MarkerArray>("planning/frenet_path", 1);///////no ros::Publisher!!!!!
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), obs_sub, r_path_sub, carstate_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  ros::Rate r(10);
  ros::spin();
  // while(nh.ok())
  // {
  //   if(frenet_c_point[0] == frenet_goal_point[0])
  //   {
  //       std::cout << "***************************************************" << std::endl;
  //       std::cout << std::endl;
  //       std::cout << "               arrvied at goal point               " << std::endl;
  //       std::cout << std::endl;
  //       std::cout << "***************************************************" << std::endl;
  //       break;
  //   }
  //   r.sleep();
  //   ros::spinOnce();
  // }
  
  // //FOR CONTROL
  // self.carstate_publisher = rospy.Publisher('/estimation/slam/state', CarState, queue_size=1)
  // //FOR CONTROL
  // self.trajectory_publisher = rospy.Publisher('/planning/TrajectoryPoint', Trajectory, queue_size=1)
  return 0;
}
