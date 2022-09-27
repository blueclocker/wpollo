
#include <iostream>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include "matplotlibcpp.h"
#include "VehicleData.h"
#include "time.h"
 #include<ctime>

//用于路径规划
#include "cubic_spline.h"
#include "frenet_path.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"

//用于碰撞检测
#include "obs_ver_list.h"
#include "collision.h"
#include "shape.h"
#include "vector2d.h"
#include <math.h>

#define SIM_LOOP 500
#define MAX_SPEED 50.0 / 3.6    // maximum speed [m/s]
#define MAX_ACCEL 4.0           // maximum acceleration [m/ss]
#define MAX_ACCEL_D -4.0           // maximum -acceleration [m/ss]
#define MAX_CURVATURE 1.0       // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 3.5      // maximum road width [m]
#define D_ROAD_W 0.1            // road width sampling length [m]
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
#define veh_length 4.0  //自车长度 （m）
#define tau 2.0  //车头实距时间常数（s）

using namespace cpprobotics;

namespace plt = matplotlibcpp;

//图形形状输出
std::vector<std::vector<double>> show_shape (std::vector<double>  a,std::vector<double>  b ){ //输入端点，输出图形
// std::cout<< "进入输出边的函数" << std::endl;
std::vector<double> self_x ;
std::vector<double> self_y;
self_x.assign(a.begin(), a.end()); 
self_y.assign(b.begin(), b.end());
std::vector<std::vector<double>> self_line(20);  //plt测试
// std::cout<< "设置完成" << std::endl;
int i = 0;

for( i= 0 ; i < self_x.size()-1 ; i++){
self_line[2*i]={{self_x[i],self_x[i+1]}}; 
self_line[2*i+1]={{self_y[i],self_y[i+1]}}; 
};
i = self_x.size()-1;
self_line[2*i]={{self_x[i],self_x[0]}}; 
self_line[2*i+1]={{self_y[i],self_y[0]}}; 
// std::cout<< "数据完成" << std::endl;

// std::cout<< "完成输出边的函数" << std::endl;
return self_line;
};

//frenet到笛卡尔坐标系转化，可以生效
Vec_f frenet_to_cartesian(float s, float d, float d_d,  Vec_f r_x,Vec_f r_y,Vec_f ryaw,Vec_f rcurvature,Vec_f rs) //frenet坐标系到笛卡尔坐标系
    {//std::cout << "进入f_d坐标转化函数 " << std::endl;
    float x_ref= 0., y_ref= 0., theta_ref= 0., k_ref =  0.;
    Vec_f cartesian_point;
    float xx, yx,theta_x;
    for (int i = 0 ; i <rs.size()-1 ; i++ ){
        if( (rs[i] - s) * (rs[i + 1] - s) <= 0.){
            x_ref = r_x[i];
            y_ref = r_y[i];
            theta_ref =ryaw[i];
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
    // std::cout << "完成坐标转化函数 " << std::endl;
    return cartesian_point;
    };

//笛卡尔坐标系到Frenet坐标系的转化,可以生效
Vec_f cartesian_to_frenet(float xx,float  yx, Vec_f r_x,Vec_f r_y,Vec_f ryaw,Vec_f rs){//笛卡尔坐标系到frenet坐标系
    // reference_path = ref_paths[lane_number].csp
    // reference_path_x = ref_paths[lane_number].x
    // reference_path_y = ref_paths[lane_number].y
    // reference_path_yaw = ref_paths[lane_number].yaw
    // index = len(reference_path_x) - 1
    float sr;
    float dot_pro1;
    float dot_pro2;
    float absolute_d;
    float d; 
    int index;
    for(int i=0; i< r_x.size() - 2;i++){ //通过这个方法找到邻近点
        dot_pro1 = (xx -  r_x[i]) * ( r_x[i + 1] -  r_x[i]) + (yx -  r_y[i]) * ( r_y[i + 1] -  r_y[i]);
        dot_pro2 = (xx -  r_x[i + 1]) * ( r_x[i + 2] -  r_x[i + 1]) + (yx -  r_y[i + 1]) * ( r_y[i + 2] -  r_y[i + 1]);
        if (dot_pro1 * dot_pro2 <= 0)
           { index = i + 1;
            break;};
    };
    sr = rs[index];
    absolute_d = pow((pow((xx - r_x[index]) , 2) + pow((yx - r_y[index]) ,2)),0.5);
    int A;
    if (((yx -r_y[index]) * cos(ryaw[index])- (xx - r_x[index]) * sin(ryaw[index]))>0)
      A = 1;
      else if (((yx -r_y[index]) * cos(ryaw[index])- (xx - r_x[index]) * sin(ryaw[index]))<0)
    A = -1;
    else A = 0;
    d = A * absolute_d;
    Vec_f  point =  {sr, d};
    return point;
};//先找最近点，然后求出世纪距离d，近似求出垂直距离d

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
int get_action (float s0 ,float s_goal , float other_vehicle_s )
{  
  // action: 0-巡航；1-停车；2-跟车
  int Action = 0;
  //还没有获取到停车点
if (s_goal -s0> 30 || s_goal == 0)
{  
  if (other_vehicle_s-s0 > 25)
  Action = 0;
  if ((other_vehicle_s>s0)&&(other_vehicle_s - s0 <= 25))
  Action = 2; 
}
  //获取到停车点
  else if (s_goal - s0 <=30)
  {  
  if (other_vehicle_s>s0)
  {if(other_vehicle_s<=s_goal)
  Action = 2;
  if (s_goal<other_vehicle_s)
  Action = 1;
  };
  if(other_vehicle_s<=s0)
  Action = 1;
  }
  else Action = 0;
  return Action;
};

//巡航下路径采样
Vec_Path calc_frenet_paths_cruising(  
    float t_speed,float c_speed, float c_d, float c_d_d, float c_d_dd, float s0)
{
  Vec_Path fp_list;
  for (float di = -(0.5 * MAX_ROAD_WIDTH-0.5*veh_width); di < (0.5 * MAX_ROAD_WIDTH-0.5*veh_width); di += D_ROAD_W)
  { 

    for (float Ti = MINT; Ti < MAXT; Ti += DT)
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
      float min_speed = t_speed - D_T_S * N_S_SAMPLE;
      if (min_speed<=0)
      min_speed = 0.0;
      for (float tv = min_speed; 
           tv < t_speed+D_T_S * N_S_SAMPLE;
           tv += D_T_S)
      {

        FrenetPath fp_bot = fp;
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();

        fp_bot.max_accel = std::numeric_limits<float>::min();
       fp_bot.max_accel_d =0;  //求最大减速度
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
         for (int j=0 ;j<fp_bot.s_dd.size();j++)
          {
            if (fp_bot.s_dd[j]<=fp_bot.max_accel_d)
           fp_bot.max_accel_d=fp_bot.s_dd[j] ;
          }
          //求最大负偏移d
         for (int j=0 ;j<fp_bot.d.size();j++)
          {
            if (fp_bot.d[j]<=fp_bot.max_right_d)
           fp_bot.max_right_d=fp_bot.d[j] ;
          }
        float Jp = sum_of_power(fp.d_ddd);
        float Js = sum_of_power(fp_bot.s_ddd);
        float ds = (t_speed - fp_bot.s_d.back());

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
        fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
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
     float tar_speed,float c_speed, float c_acc,float c_d, float c_d_d, float c_d_dd, float s0,float frenet_s_goal,float frenet_d_goal)
{
  Vec_Path fp_list;
  for (float di = frenet_d_goal; di < frenet_d_goal+0.01; di += 0.5*D_ROAD_W)
  { // MAX_ROAD_WIDTH  7.0   D_ROAD_W  1.0
   //设定最小停车时间
 float mint = MINT;
  if (c_speed/(-MAX_ACCEL_D) < mint )
  mint = c_speed/(-MAX_ACCEL_D);
  //设定最大停车时间
  float maxt= MAXT;
  if (( 10/(c_speed+0.1)) > maxt )
  maxt = ( 10/(c_speed+0.1));
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
      for (float ts = frenet_s_goal-4; 
           ts < frenet_s_goal+0.01;
           ts += 1)
      {

        FrenetPath fp_bot = fp;
        QuinticPolynomial lon_qp(s0, c_speed, c_acc, ts,0, 0.0,Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
       fp_bot.max_accel_d =0;  //求最大减速度
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
         for (int j=0 ;j<fp_bot.s_dd.size();j++)
          {
            if (fp_bot.s_dd[j]<=fp_bot.max_accel_d)
           fp_bot.max_accel_d=fp_bot.s_dd[j] ;
          }
                              //求最大负偏移d
         for (int j=0 ;j<fp_bot.d.size();j++)
          {
            if (fp_bot.d[j]<=fp_bot.max_right_d)
           fp_bot.max_right_d=fp_bot.d[j] ;
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
    float c_speed, float c_acc,float c_d, float c_d_d, float c_d_dd, float s0,float followVeh_s,float followVeh_d,float followVeh_s_v,float followVeh_d_v)
{
  Vec_Path fp_list;
  for (float di = -(0.5 * MAX_ROAD_WIDTH-0.5*veh_width); di < 0.5 * MAX_ROAD_WIDTH-0.5*veh_width; di += D_ROAD_W)
  { // D_ROAD_W  1.0
   float mint = MINT;
  if (c_speed/(-MAX_ACCEL_D) < mint )
  mint = c_speed/(-MAX_ACCEL_D);
  //设定最大停车时间
    for (float Ti = mint; Ti < MAXT+1; Ti += DT)
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
     float s_fv1 = followVeh_s + followVeh_s_v * Ti+0.5*0*std::pow(Ti,2);
     float s_target = s_fv1 - tau*fabs(c_speed-followVeh_s_v)-10;
     float s_target_d =followVeh_s_v+0*Ti;
     float s_target_dd =0;
     if (s_target > s0 + c_speed * Ti + 0.5 * MAX_ACCEL *std::pow(Ti,2))
      s_target = s0 + 0.5 * MAX_ACCEL*std::pow(Ti,2);
     if (s_target_d > MAX_SPEED)
     s_target_d = 0.98 * MAX_SPEED;
      if (s_target_dd > MAX_ACCEL)
     s_target_dd = 0.98 * MAX_ACCEL;

      FrenetPath fp_bot = fp;
      QuinticPolynomial lon_qp(s0, c_speed, c_acc,s_target, s_target_d, s_target_dd, Ti);

      fp_bot.max_speed = std::numeric_limits<float>::min();
      fp_bot.max_accel = std::numeric_limits<float>::min();
      fp_bot.max_accel_d = 0;  //求最大减速度
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
         for (int j=0 ;j<fp_bot.s_dd.size();j++)
          {
            if (fp_bot.s_dd[j]<=fp_bot.max_accel_d)
           fp_bot.max_accel_d=fp_bot.s_dd[j] ;
          }
               //               //求最大负偏移d
         for (int j=0 ;j<fp_bot.d.size();j++)
          {
            if (fp_bot.d[j]<=fp_bot.max_right_d)
           fp_bot.max_right_d=fp_bot.d[j] ;
            }

        float Jp = sum_of_power(fp.d_ddd);
        float Js = sum_of_power(fp_bot.s_ddd);

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back()-followVeh_d, 2);
        fp_bot.cv = KJ * Js + KT * Ti ;
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

        fp_list.push_back(fp_bot);
    }
  }
    std::cout << "跟车时采样路径数目:" << fp_list.size() << std::endl;
  return fp_list;
}


//停车下的路径采样,第二版
Vec_Path calc_frenet_paths_stopping(  
    float c_speed, float c_acc,float c_d, float c_d_d, float c_d_dd, float s0,float frenet_s_goal,float frenet_d_goal)
{
Vec_Path  fp_list;
  float s_goal = frenet_s_goal;
  float d_goal = frenet_d_goal;
 //设定最小停车时间
 float mint = MINT;
  if (c_speed/(-MAX_ACCEL_D) < mint )
  mint = c_speed/(-MAX_ACCEL_D);
  //设定最大停车时间
  float maxt= MAXT;
  if (( 2*(s_goal-s0)/(c_speed+0.1)) > maxt )
  maxt = ( 2*(s_goal-s0)/(c_speed+0.1));
  if (maxt >= 6)
   maxt = 6;
    for (float Ti = mint; Ti<maxt; Ti += DT)
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

      FrenetPath   fp_bot = fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, d_goal, 0.0, 0.0, Ti);
      fp_bot.max_speed = std::numeric_limits<float>::min();
      fp_bot.max_accel = std::numeric_limits<float>::min();
      // fp_bot.max_accel_d =fabs(-std::numeric_limits<float>::min());  //求最大减速度的有问题
      fp_bot.max_accel_d = 0 ;
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
                  for (int j=0 ;j<fp_bot.s_dd.size();j++)
          {
            if (fp_bot.s_dd[j]<=fp_bot.max_accel_d)
           fp_bot.max_accel_d=fp_bot.s_dd[j] ;
          }
                              //求最大负偏移d
         for (int j=0 ;j<fp_bot.d.size();j++)
          {
            if (fp_bot.d[j]<=fp_bot.max_right_d)
           fp_bot.max_right_d=fp_bot.d[j] ;
            }
      }
       float Jp = sum_of_power(fp.d_ddd);
       float Js = sum_of_power(fp_bot.s_ddd);
       float ds = s_goal-frenet_s_goal ;
        // fp_bot.cd = KJ * Jp + KT * Ti ;
        // fp_bot.cv = KJ * Js + KT * Ti +10 * KD * ds;

        fp_bot.cd = -KT * Ti ;
        fp_bot.cv =  -KT * Ti ;
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
    for (int i = 0; i < path_p->x.size() - 1; i++)
    {       
      float dx = path_p->x[i + 1] - path_p->x[i];
      float dy = path_p->y[i + 1] - path_p->y[i];
      path_p->yaw.push_back(std::atan2(dy, dx)); //返回弧度值
      path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
    }
    path_p->yaw.push_back(path_p->yaw.back());
    path_p->ds.push_back(path_p->ds.back());
    path_p->max_curvature = std::numeric_limits<float>::min();
    for (int i = 0; i < path_p->x.size() - 1; i++)
    {          
      path_p->c.push_back((path_p->yaw[i + 1] - path_p->yaw[i]) / path_p->ds[i]);
      if (path_p->c.back() > path_p->max_curvature)
      {     
        path_p->max_curvature = path_p->c.back();
      }
    }
  }
};

//碰撞检测
bool check_collision(FrenetPath path, const Polygon_list obs_list)  
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
 
 for(i=0;i<path.x.size();i++)
{
  self_x00 = path.x[i] + 0.5 * veh_length * cos(path.yaw[i]); //车头中间点
      self_y00 = path.y[i] + 0.5 * veh_length * sin(path.yaw[i]);
      self_x01 = path.x[i] - 0.5 * veh_length * cos(path.yaw[i]); //车尾中间点
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

  for (auto obs:obs_list)
    // for (int j=0;j<obs_list.size(); j++)
  {
    Simplex simplex;
    if (intersect(polygon_veh, obs))
    return true;
    else continue;
  }
}
  }
  return false;
};

//动态障碍物碰撞检测
bool dynamic_check_collision(FrenetPath path, const Polygon_list obs_list)  
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
 
 for(i=0;i<5;i++)
{
  self_x00 = path.x[i] + 0.5 * veh_length * cos(path.yaw[i]); //车头中间点
      self_y00 = path.y[i] + 0.5 * veh_length * sin(path.yaw[i]);
      self_x01 = path.x[i] - 0.5 * veh_length * cos(path.yaw[i]); //车尾中间点
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

  for (auto obs:obs_list)
  {
    Simplex simplex;
    if (intersect(polygon_veh, obs))
    return true;
    else continue;
  }
}
  return false;
};

//路径筛选
Vec_Path check_paths(Vec_Path path_list, const Polygon_list obs_list)  
{
  std::cout<<"进入路径筛选"<<std::endl;
  Vec_Path output_fp_list;
  for (FrenetPath path : path_list)
  {
    if (path.max_left_d < 4.3 &&path.max_right_d >-0.7 &&path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL &&path.max_accel_d >MAX_ACCEL_D  && path.max_curvature < MAX_CURVATURE && !check_collision(path, obs_list))
    {

      output_fp_list.push_back(path);
    }
  }
  std::cout<<"有可用路径 "<< output_fp_list.size()<<" 条。"<<std::endl;
  return output_fp_list;
};
//有动态障碍下的路径筛选
Vec_Path check_paths_dynamic(Vec_Path path_list, const Polygon_list obs_list)  
{
  Vec_Path output_fp_list;
  for (FrenetPath path : path_list)
  {
    if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL &&path.max_accel_d >MAX_ACCEL_D  && path.max_curvature < MAX_CURVATURE && !check_collision(path, obs_list))
    {
      output_fp_list.push_back(path);
    }
  }
  return output_fp_list;
};
//无障碍物下的路径筛选
Vec_Path check_paths_nobs(Vec_Path path_list, const Polygon_list obs_list)  
{
  Vec_Path output_fp_list;
  for (FrenetPath path : path_list)
  {
    if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL &&path.max_accel_d >MAX_ACCEL_D  && path.max_curvature < MAX_CURVATURE )
    {
      output_fp_list.push_back(path);
    }
  }
  return output_fp_list;
};

FrenetPath frenet_optimal_planning_cruising(  //巡航下路径规划
    float T_speed,Spline2D csp, float s0, float c_speed,
    float c_d, float c_d_d, float c_d_dd, Polygon_list obs_list)
{
  Vec_Path fp_list = calc_frenet_paths_cruising(T_speed,c_speed, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fp_list, csp);
  Vec_Path save_paths = check_paths(fp_list, obs_list);
  if (save_paths.empty() )
  std::cout<<"没有采样出最优路径"<<std::endl;
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
  std::cout << "输出巡航时最优路径" <<std::endl;
  return final_path;
};

FrenetPath frenet_optimal_planning_stopping(  //停车下路径规划
    float T_speed,Spline2D csp, float s0, float c_speed,float c_acc,
    float c_d, float c_d_d, float c_d_dd, float s_goal,float d_goal,Polygon_list obs_list,FrenetPath last_path)
{    //std::cout << "进入到停车时采样第一层函数" <<std::endl;
  Vec_Path fp_list = calc_frenet_paths_stopping(c_speed, c_acc,c_d, c_d_d, c_d_dd, s0,s_goal,d_goal);
      // std::cout << "完成第一部采样" <<std::endl;
  calc_global_paths(fp_list, csp);
    // std::cout << "完成路径直角坐标系信息补充" <<std::endl;
  Vec_Path save_paths = check_paths(fp_list, obs_list);
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
std::cout << "输出停车时最优路径" <<std::endl;
return final_path;
};

//没路时自己设停车点,测试修改版
FrenetPath frenet_optimal_planning_stopping_noway(  
    float T_speed,Spline2D csp, float s0, float c_speed,float c_acc,
    float c_d, float c_d_d, float c_d_dd, float s_goal,float d_goal,Polygon_list obs_list,FrenetPath last_path)
{    std::cout << "进入没路停车时采样第一层函数" <<std::endl;
  Vec_Path fp_list = calc_frenet_paths_stopping_noway(T_speed,c_speed, c_acc,c_d, c_d_d, c_d_dd, s0,s_goal,d_goal);
      // std::cout << "完成第一部采样" <<std::endl;
  calc_global_paths(fp_list, csp);
  Vec_Path save_paths = check_paths(fp_list, obs_list);
  FrenetPath final_path;
  float min_cost = std::numeric_limits<float>::max();
  for (auto path : save_paths)
  {
    if (min_cost >= path.cf)
    {
      min_cost = path.cf;
      final_path = path;
    }
  std::cout << "输出自定义停车时最优路径" <<std::endl;
  return final_path;
  }

  // if (final_path.s.empty()  )
  // final_path = last_path;  
std::cout << "输出没路时停车时最优路径" <<std::endl;
return final_path;
};

//跟车下路径规划
  FrenetPath frenet_optimal_planning_following(  
    float T_speed,Spline2D csp, float s0, float c_speed,float c_acc,
    float c_d, float c_d_d, float c_d_dd,float ThefollowVeh_s,float ThefollowVeh_d,float THefollowVeh_s_v,float ThefollowVeh_d_v,Polygon_list obs_list,FrenetPath last_path)
{    //std::cout << "进入到停车时采样第一层函数" <<std::endl;
  Vec_Path fp_list = calc_frenet_paths_following(c_speed, c_acc,c_d,  c_d_d, c_d_dd,  s0,ThefollowVeh_s,ThefollowVeh_d,THefollowVeh_s_v,ThefollowVeh_d_v);
      // std::cout << "完成第一部采样" <<std::endl;
  calc_global_paths(fp_list, csp);
    // std::cout << "完成路径直角坐标系信息补充" <<std::endl;
  Vec_Path save_paths = check_paths(fp_list, obs_list);
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
std::cout << "输出跟车时最优路径" <<std::endl;
return final_path;
};

//行车位置预测
Vec_f LocationPrediction(float now_s, float now_d, float s_speed,float d_speed)
{
  Vec_f NextLocation;
  float next_s = now_s + s_speed*DT;
  float next_d = now_d + d_speed*DT;
  NextLocation.push_back(next_s);
  NextLocation.push_back(next_d);
  return NextLocation;
};

////主程序开始
int main()
{
//定义目标速度
float target_speed=TARGET_SPEED;
//定义总耗时
  float TotalTime = 0.0;
//是否有自己设置的停车路径
int show_path = 0;
  //参考路径
  Vec_f wx({0.0, 10.0, 20.5, 35.0, 70.5,90,120,150,200}); 
  Vec_f wy({0.0,0,0,0,0,0,0,0,0});

//版本更改
  Simplex simplex;
  //这个是演示程序输出障碍物包围盒的列表
std::vector<std::vector<double>> obstcles_list_vertex;  
//新加的障碍物列表
ListObsVer Obstacle_Vertexes_list;
//此列表里存放着class ObstacleVertexes（obs_ver_list.h里定义的），每一个ObstacleVertexes存放着端点信息
for(int j = 0 ;j<6 ; j++)
{//这个是我为了体现循环自己加的一个变化量
  float suibian = 10*j;
  //新开辟一个单一障碍物端点列表
  ObstacleVertexes *OBSvertexes = new ObstacleVertexes;
  //依次存放进取，存放几个端点可以自己修改
  OBSvertexes->vertexes_list.push_back({50+suibian,0.4});
  OBSvertexes->vertexes_list.push_back({54+suibian,0.4});
  OBSvertexes->vertexes_list.push_back({54+suibian,2.7});
  OBSvertexes->vertexes_list.push_back({50+suibian,2.7});
  //单一障碍物端点列表存放到障碍物列表中
  Obstacle_Vertexes_list.push_back(*OBSvertexes);
};
//障碍物端点数目列表
std::vector<int> num_vertices_obstcle;
//
  std::vector<Vector2D*> vector_obstcle;
  //判断是否对该障碍物进行避障检测所需的距离判断列表
  Vec_f*vector_obstcle_center  =  new Vec_f ;
//循环开始，对于障碍物列表里每一个障碍物端点信息
for(int j=0;j<Obstacle_Vertexes_list.size();j++)
{
  num_vertices_obstcle.push_back(Obstacle_Vertexes_list[j].vertexes_list.size());
  Vector2D *vector_obstcle_data = new Vector2D[num_vertices_obstcle[j]];
  for (int z=0;z<num_vertices_obstcle[j];z++)
  {//在避障信息列表里存入点信息
    vector_obstcle_data[z] = Vector2D(Obstacle_Vertexes_list[j].vertexes_list[z][0],Obstacle_Vertexes_list[j].vertexes_list[z][1]);
  }
  //存入判断距离信息（为障碍物第一个端点的x值，后续要修改）
  vector_obstcle_center->push_back(Obstacle_Vertexes_list[j].vertexes_list[0][0]);
  //碰撞障碍物列表里存放障碍物
vector_obstcle.push_back(vector_obstcle_data);
}
//为了画图，录入画图信息
for(int j = 0 ;j<6 ; j++)
{float suibian = 10*j;
obstcles_list_vertex.push_back({50+suibian,54+suibian,54+suibian,50+suibian}); 
obstcles_list_vertex.push_back({0.4,0.4,2.7,2.7});
}

//参考路径信息
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
 
 //初始信息
  float c_speed = 35.0 / 3.6; 
   float c_acc = 0; 
  float c_d = 0.5;
  float c_d_d = 0.0;
  float c_d_dd = 0.0;
  float s0 = 0.0;
  float s_goal_give = 170;  //停车目标点
  float d_goal_give = -0.5;
// std::cout << "开始求终点直角坐标 " << std::endl;
Vec_f  cartesian_goal_point = frenet_to_cartesian(s_goal_give, d_goal_give, 0, r_x,r_y,ryaw,rcurvature,rs); //求终点直角坐标系信息
std::cout << "s = " << s_goal_give << " , d = " << d_goal_give << " , x = " <<  cartesian_goal_point[0] <<  " , y = " <<  cartesian_goal_point[1] << std::endl;

//意图代号，0-巡航，1-停车，2-跟车
  int action = 0;

  float current_s = 0;

//自车所需信息
  float self_x00; 
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

    FrenetPath final_path;
    FrenetPath alter_final_path;
    FrenetPath final_path_show;
    FrenetPath last_path;

  //是否调用上次路径的计数器
    int count_last_path = 0 ;
     
    int  stop_and_restart = 0;
    //其他行车初始信息
    // float otherVeh_1_s = 40.0;
        float otherVeh_1_s = 0;
  //   float otherVeh_1_d = -3.0;
  //   float otherVeh_1_v_s = 15.0/3.6;
  //   float otherVeh_1_v_d = 0;
  //   float otherVeh_1_v_d_d = 0;
  //   //  Vec_f frenet_to_cartesian(float s, float d, float d_d,  Vec_f r_x,Vec_f r_y,Vec_f ryaw,Vec_f rcurvature,Vec_f rs) 
  //  Vec_f Vehicle_1_base = frenet_to_cartesian(otherVeh_1_s, otherVeh_1_d, otherVeh_1_v_d_d, r_x,r_y,ryaw,rcurvature,rs);
  //    Vec_f Vehicle_1 = {TotalTime,otherVeh_1_s,otherVeh_1_d,otherVeh_1_v_s,otherVeh_1_v_d,Vehicle_1_base[0],Vehicle_1_base[1],2.0,4.0,Vehicle_1_base[2]};
  //定义显示循环时间的一些变量
  int loop_num = 0;
 clock_t startTime,endTime;
  //开始最主要的循环环节
//障碍物储存列表
  Polygon_list obstcles_list;
  for (int i = 0; i < SIM_LOOP; i++)
  { 
    show_path = 0;
    //记时开始,记录循环时间
    startTime = clock();
    loop_num ++;
   std::cout << "开始第 "<< i + 1 << "次规划" <<std::endl;
   stop_and_restart = 0;
//障碍物列表更新
std::cout << "进入列表更新 " <<std::endl;
for(int k = 0;k<num_vertices_obstcle.size();k++)
{     //我只要对在我碰撞检测范围内的障碍物进行避障检测
float check_dis = 50;
if (c_speed*MAXT> check_dis)
check_dis = MAXT*c_speed;
  if( ((vector_obstcle_center->at(k)>=s0)&&(vector_obstcle_center->at(k)-s0<=check_dis))||  ((vector_obstcle_center->at(k)<s0)&&(s0-vector_obstcle_center->at(k)<20)) )
 { obstcles_list .push_back(Polygon(num_vertices_obstcle.at(k), vector_obstcle.at(k))); //用于碰撞检测
 }
  // obstcles_list.push_back( Polygon(num_vertices_obstcle_2, vector_obstcle_2));
  // obstcles_list.push_back (Polygon(num_vertices_obstcle_3, vector_obstcle_3));
  // obstcles_list.push_back(Polygon(num_vertices_obstcle_4, vector_obstcle_4));
  // obstcles_list.push_back(Polygon(num_vertices_obstcle_5, vector_obstcle_5));
  // obstcles_list.push_back(Polygon(num_vertices_obstcle_6, vector_obstcle_6));
  // obstcles_list.push_back(Polygon(num_vertices_obstcle_7, vector_obstcle_7));
  // obstcles_list.push_back(Polygon(num_vertices_obstcle_8, vector_obstcle_8));
  // obstcles_list.push_back(Polygon(num_vertices_obstcle_9, vector_obstcle_9));

}

   //给其他行车信息

     // 函数名为 VehicleData::VehicleData (float t,float s, float d, float v_s,float v_d,float x, float y, float width,float length,float yaw)
  //  VehicleData OtherVehicle_1(Vehicle_1[0],Vehicle_1[1],Vehicle_1[2],Vehicle_1[3],Vehicle_1[4],Vehicle_1[5],Vehicle_1[6],Vehicle_1[7],Vehicle_1[8],Vehicle_1[9]);
   
   //添加其它车的碰撞信息
  //    int num_vertices_OtherVeh_1 = 4;
  //  Vec_f veh1_x_line_float = OtherVehicle_1.Vehicle_x();
  //  Vec_f veh1_y_line_float = OtherVehicle_1.Vehicle_y();

  // Vector2D *vector_OtherVeh_1 = new Vector2D[num_vertices_OtherVeh_1];

  // vector_OtherVeh_1[0] = Vector2D(veh1_x_line_float[0], veh1_y_line_float[0]);
  // vector_OtherVeh_1[1] = Vector2D(veh1_x_line_float[1], veh1_y_line_float[1]);
  // vector_OtherVeh_1[2] = Vector2D(veh1_x_line_float[2],  veh1_y_line_float[2]);
  // vector_OtherVeh_1[3] = Vector2D(veh1_x_line_float[3],  veh1_y_line_float[3]);
  
  //   obstcles_list.push_back(Polygon(num_vertices_OtherVeh_1, vector_OtherVeh_1));
   
    //简化意图获取
  action = get_action (s0 ,s_goal_give ,otherVeh_1_s);
  if (action == 0 ) std::cout << "意图为巡航" << std::endl;
 if (action == 1 ) std::cout << "意图为停车" << std::endl;

    // if (s_goal_give -s0> 20 || s_goal_give == 0 ){  
   if (action == 0 ){  
      std:: cout << "no_stopping     i="<< i << std::endl;
final_path = frenet_optimal_planning_cruising(
        target_speed,csp_obj, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles_list);
    }

    else if (action == 1) {
      std:: cout << "get_stopping     i="<< i << std::endl;
final_path = frenet_optimal_planning_stopping(
       target_speed,csp_obj, s0, c_speed,c_acc, c_d, c_d_d, c_d_dd, s_goal_give,d_goal_give,obstcles_list,last_path);
  std::cout << "完成停车路径采样和信息补充" <<std::endl;
  };

//       else if (action == 2) {
//       std:: cout << "get_following     i="<< i << std::endl;
// final_path = frenet_optimal_planning_following(
//        target_speed,csp_obj, s0, c_speed,c_acc, c_d, c_d_d, c_d_dd, otherVeh_1_s,otherVeh_1_d,otherVeh_1_v_s,otherVeh_1_v_d,obstcles_list,last_path);
//   std::cout << "完成跟车路径采样和信息补充" <<std::endl;
//   }

//  else break;

 //如果上次没有规划出来路径，判断是否调用上次最优路径
  if (final_path.s.empty()&&last_path.s.size()>2&&( count_last_path<5) )
{   
  std::cout << "准备调用上次路径" << std::endl;
  if(std::pow(s0 -last_path.s.back(),2)<0.2)
  {
    std::cout<<"到达设定点1"<<std::endl;
    break;}
  // float target_speed_noway = 10/3.6;  //没有路的时候目标速度给低一点
  // target_speed = TARGET_SPEED/(1+100*final_path.max_curvature);
  target_speed = TARGET_SPEED; 
  // if (target_speed > target_speed_noway)
  // target_speed = target_speed_noway;
  last_path.s.erase(last_path.s.begin() + 0);
  last_path.s_d.erase(last_path.s_d.begin() + 0);
  last_path.s_dd.erase(last_path.s_dd.begin() + 0);
  last_path.s_ddd.erase(last_path.s_ddd.begin() + 0);
   last_path.t.erase(last_path.t.begin() + last_path.t.size()-1);
  last_path.d.erase(last_path.d.begin() + 0);
  last_path.d_d.erase(last_path.d_d.begin() + 0);
  last_path.d_dd.erase(last_path.d_dd.begin() + 0);
  last_path.d_ddd.erase(last_path.d_ddd.begin() + 0);
  last_path.x.erase(last_path.x.begin() + 0);
  last_path.y.erase(last_path.y.begin() + 0);
  last_path.yaw.erase(last_path.yaw.begin() + 0);
 last_path.ds.erase(last_path.ds.begin() + 0);
  last_path.c.erase(last_path.c.begin() + 0);
  count_last_path +=1 ;
std::cout << "第 " << count_last_path << " 次调用上一次最优路径 。" << std::endl;
 s0 = last_path.s[1];
 c_d = last_path.d[1];
 c_d_d = last_path.d_d[1];
  c_d_dd = last_path.d_dd[1];
  c_speed = last_path.s_d[1];
  c_acc =last_path.s_dd[1];
if (!check_collision(last_path, obstcles_list))
  final_path = last_path;  
  else 
  {
   s0 = last_path.s[0];
   c_d = last_path.d[0];
   c_d_d = last_path.d_d[0];
   c_d_dd = last_path.d_dd[0];
   c_speed = last_path.s_d[0];
   c_acc = last_path.s_dd[0];
   alter_final_path = frenet_optimal_planning_stopping_noway(
  target_speed,csp_obj, s0, c_speed,c_acc, c_d, c_d_d, c_d_dd, last_path.s.back(),last_path.d.back(),obstcles_list,last_path);
    if (!alter_final_path.s.empty())
  {
    std::cout<<"有暂时停车路径" <<std::endl;
    show_path = 1;
    final_path = alter_final_path;
     if(std::pow(s0 -final_path.s.back(),2)<0.2)
  {
    std::cout<<"到达设定点2"<<std::endl;
    break;}
  }
  else 
  {
    std::cout<<"没找到其他可行路!!建议停车!!stop!stop!stop!--1"<<std::endl;
    break;
  }
  }
}
else if(final_path.s.empty()&&last_path.s.size()>2&&( count_last_path>=5))
{
    std::cout << "无法调用上次路径,准备直接生成停车路径" << std::endl;
  last_path.s.erase(last_path.s.begin() + 0);
  last_path.s_d.erase(last_path.s_d.begin() + 0);
  last_path.s_dd.erase(last_path.s_dd.begin() + 0);
  last_path.s_ddd.erase(last_path.s_ddd.begin() + 0);
   last_path.t.erase(last_path.t.begin() + last_path.t.size()-1);
  last_path.d.erase(last_path.d.begin() + 0);
  last_path.d_d.erase(last_path.d_d.begin() + 0);
  last_path.d_dd.erase(last_path.d_dd.begin() + 0);
  last_path.d_ddd.erase(last_path.d_ddd.begin() + 0);
  last_path.x.erase(last_path.x.begin() + 0);
  last_path.y.erase(last_path.y.begin() + 0);
  last_path.yaw.erase(last_path.yaw.begin() + 0);
 last_path.ds.erase(last_path.ds.begin() + 0);
  last_path.c.erase(last_path.c.begin() + 0);
   s0 = last_path.s[0];
   c_d = last_path.d[0];
   c_d_d = last_path.d_d[0];
   c_d_dd = last_path.d_dd[0];
   c_speed = last_path.s_d[0];
   c_acc = last_path.s_dd[0];
   alter_final_path = frenet_optimal_planning_stopping_noway(
  target_speed,csp_obj, s0, c_speed,c_acc, c_d, c_d_d, c_d_dd, last_path.s.back(),last_path.d.back(),obstcles_list,last_path);
    if (!alter_final_path.s.empty())
  {
    std::cout<<"有暂时停车路径" <<std::endl;
    show_path = 1;
    final_path = alter_final_path;
    if(std::pow(s0 -final_path.s.back(),2)<0.2)
  {
    std::cout<<"到达设定点3"<<std::endl;
    break;}
  }
  else 
  {
    std::cout<<"没找到其他可行路!!建议停车!!stop!stop!stop!--2"<<std::endl;
    break;
  }
}
else if(final_path.s.empty()&&last_path.s.size()<=2)
{
    std::cout<<"都不行,没找到其他可行路!!建议停车!!stop!stop!stop!--3"<<std::endl;
   s0 = last_path.s[1];
   c_d = last_path.d[1];
   c_d_d = last_path.d_d[1];
   c_d_dd = last_path.d_dd[1];
   c_speed = last_path.s_d[1];
   c_acc = last_path.s_dd[1];
    break;
}
else 
{
    std::cout << "规划成功" << std::endl;
count_last_path=0;
target_speed = TARGET_SPEED;
 s0 = final_path.s[1];
 c_d = final_path.d[1];
 c_d_d = final_path.d_d[1];
  c_d_dd = final_path.d_dd[1];
  c_speed = final_path.s_d[1];
  c_acc = final_path.s_dd[1];
 }

if (final_path.s.size()<=2)
{
  stop_and_restart = 1;
};


last_path = final_path;

final_path_show = final_path;

  std::cout << "下一次循环信息: s0 = " << s0 << "   ,  d0 =  " << c_d << " , 纵向速度 = "<< c_speed<< std::endl;
//记录下障碍物数目
  std::string obstcle_data= "num of obstcles  =    " + std::to_string(obstcles_list.size());
    //清碰撞检测列表
    obstcles_list.clear();
   //删除碰撞列表添加的其它车信息
    // obstcles_list.erase(obstcles_list.end()-1);

    //自车位置
    // for (int i = 0; i < final_path_show.x.size(); i++) {  //自车位置循环版
        for (int i = 0; i < 1; i++) {  
            // std::cout << "开始输出图像" <<std::endl;
      self_x00 = final_path_show.x[i] + 0.5 * veh_length * cos(final_path_show.yaw[i]); //车头中间点
      self_y00 = final_path_show.y[i] + 0.5 * veh_length * sin(final_path_show.yaw[i]);
      self_x01 = final_path_show.x[i] - 0.5 * veh_length * cos(final_path_show.yaw[i]); //车尾中间点
      self_y01 = final_path_show.y[i] - 0.5 * veh_length * sin(final_path_show.yaw[i]);
      self_x1 = self_x00 + 0.5 * veh_width * cos(final_path_show.yaw[i] + 0.5 * M_PI);
      self_y1 = self_y00 + 0.5 * veh_width * sin(final_path_show.yaw[i] + 0.5 * M_PI);
      self_x2 = self_x00 + 0.5 * veh_width * cos(final_path_show.yaw[i] - 0.5 * M_PI);
      self_y2 = self_y00 + 0.5 * veh_width * sin(final_path_show.yaw[i] - 0.5 * M_PI);
      self_x3 = self_x01 + (self_x2 - self_x00);
      self_y3 = self_y01 + (self_y2 - self_y00);
      self_x4 = self_x01 + (self_x1 - self_x00);
      self_y4 = self_y01 + (self_y1 - self_y00);

  
std::vector<double> self_veh_x = {self_x1, self_x2, self_x3, self_x4};  //plt测试
std::vector<double> self_veh_y = { self_y1,self_y2, self_y3, self_y4};

endTime = clock();//计时结束
std::cout << "The run time at loop number = "<< loop_num << "  is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

plt::clf();
plt::grid(1);  //显示网格线

Vec_f goal_frenet = cartesian_to_frenet(cartesian_goal_point[0],cartesian_goal_point[1],r_x, r_y, ryaw,rs);

std::cout << "goal_s = " << goal_frenet[0] <<", goal_d = "<< goal_frenet[1] << std::endl;

//目标点画圆
    std::vector<double> goal_x(20),goal_y(20);   
    for(int i=0; i<20; ++i) {
        double t = 2*M_PI*i/20;
        goal_x.at(i) = cartesian_goal_point[0]+2.5*cos(t);
        goal_y.at(i) =cartesian_goal_point[1]+2.5*sin(t);
    };
            goal_x.push_back( cartesian_goal_point[0]+2.5*cos(0));
        goal_y.push_back( cartesian_goal_point[1]+2.5*sin(0));
  plt::plot(goal_x, goal_y, "c-");

//输出行车点
plt::scatter(self_veh_x, self_veh_y);

//画出自车包围凸体
std::vector<std::vector<double>> self_veh_line = show_shape(self_veh_x,self_veh_y); 
for(int i=0 ;i<0.5*self_veh_line.size();i++)  
plt::plot(self_veh_line[2*i],self_veh_line[2*i+1],"k");  


//画出车1
//    std::vector<double> veh1_x_line;
// std::vector<double> veh1_y_line;
// for (int m = 0;m<veh1_x_line_float.size();m++ )
// {
//   veh1_x_line.push_back((double)veh1_x_line_float[m]);
//   veh1_y_line.push_back((double)veh1_y_line_float[m]);
// };
// plt::scatter(veh1_x_line,veh1_y_line);
// std::vector<std::vector<double>> other_veh_1_line = show_shape(veh1_x_line,veh1_y_line); 
// for(int i=0 ;i<0.5*other_veh_1_line.size();i++)  
// plt::plot(other_veh_1_line[2*i],other_veh_1_line[2*i+1],"g");

//画出障碍物列表下的障碍物凸体
std::vector<std::vector<double>> obs_line;
for(int i=0 ;i<0.5*obstcles_list_vertex.size();i++) {  
std::vector<std::vector<double>> obs_line= show_shape( obstcles_list_vertex[2*i], obstcles_list_vertex[2*i+1]);
for(int j=0 ;j<0.5*obs_line.size();j++)  
plt::plot(obs_line[2*j],obs_line[2*j+1],"r-"); 
};

//输出障碍物点
for(int i = 0; i<0.5*obstcles_list_vertex.size();i++)
plt::scatter(obstcles_list_vertex[2*i],obstcles_list_vertex[2*i+1]);

//输出参考路径与道路两侧线
plt::plot(r_x, r_y,"y-");
Vec_f r_y_left_1;
Vec_f r_y_left_2;
Vec_f r_y_right_1;
for (int n = 0;n< r_y.size();n++)
{
  r_y_left_1.push_back(r_y[n]+0.5*MAX_ROAD_WIDTH);
   r_y_left_2.push_back(r_y[n]+1.5*MAX_ROAD_WIDTH);
  r_y_right_1.push_back(r_y[n]-0.5*MAX_ROAD_WIDTH);
};
plt::plot(r_x, r_y_left_1,"k-");
plt::plot(r_x, r_y_left_2,"k-");
plt::plot(r_x, r_y_right_1,"k-");

//输出最终规划路径
if(!final_path_show.x.empty())
plt::plot(final_path_show.x, final_path_show.y); 
//按要求输出自定义紧急停车时的路径
if(show_path == 1)
plt::plot(alter_final_path.x, alter_final_path.y);
plt::xlim(final_path.x.at(0)-10,final_path.x.at(0)+50) ;//窗口范围限制
plt::ylim(final_path.y.at(0)-10., final_path.y.at(0)+10.);
plt::xlabel("x (m)");
plt::ylabel("y (m)");
//设置横纵坐标轴比例尺相同
plt::set_aspect(1);
plt::title("Frenet Trajectory Optimal");
std::string speed_data= "speed =    " + std::to_string(3.6*c_speed) +"    km/h" ;
std::string Time_of_each_cycle= "Time of each cycle =    " + std::to_string((double)(endTime - startTime) / CLOCKS_PER_SEC) +"    s" ;
std::string target_speed_data= "target speed =    " + std::to_string(3.6*target_speed) +"    km/h" ;
std::string max_curvature_data= "max curvature =    " + std::to_string(final_path.max_curvature) +"    /m" ;
// std::string obstcle_data= "num of obstcles  =    " + std::to_string(obstcles_list.size());
// plt::text(final_path.x.at(0)-7, final_path.y.at(0)-7.5,max_curvature_data);
// plt::text(final_path.x.at(0)-7, final_path.y.at(0)-8,target_speed_data);
// plt::text(final_path.x.at(0)-7, final_path.y.at(0)-8.5,speed_data);
// plt::text(final_path.x.at(0)-7, final_path.y.at(0)-9,Time_of_each_cycle);
// plt::text(final_path.x.at(0)-4, final_path.y.at(0)-9.5,obstcle_data);
plt::text(final_path.x.at(0)-4, final_path.y.at(0)-9,speed_data);
plt::text(final_path.x.at(0)-4, final_path.y.at(0)-8,Time_of_each_cycle);
plt::text(final_path.x.at(0)-4, final_path.y.at(0)-7,max_curvature_data);
plt::text(final_path.x.at(0)-4, final_path.y.at(0)-6,target_speed_data);
plt::text(final_path.x.at(0)-4, final_path.y.at(0)-5,obstcle_data);
// plt::axis("equal");
plt::pause(0.0000001);

std::cout << "图像输出成功" <<std::endl;

//其他行车行为预测
// Vec_f Next_Other_1_data = LocationPrediction(otherVeh_1_s, otherVeh_1_d, otherVeh_1_v_s,otherVeh_1_v_d);
//   otherVeh_1_s = Next_Other_1_data[0];
//   otherVeh_1_d = Next_Other_1_data[1];
// Vehicle_1_base = frenet_to_cartesian(otherVeh_1_s, otherVeh_1_d, otherVeh_1_v_d_d, r_x,r_y,ryaw,rcurvature,rs);
// Vehicle_1 = {TotalTime,otherVeh_1_s,otherVeh_1_d,otherVeh_1_v_s,otherVeh_1_v_d,Vehicle_1_base[0],Vehicle_1_base[1],2.0,4.0,Vehicle_1_base[2]};
   
std::cout << "完成循环;" <<std::endl;
TotalTime += DT;
  }
  if ((std::pow((s_goal_give - final_path_show.s[0]), 2) <= 0.5&& s_goal_give != 0) || rs.back() - final_path_show.s[0] <= 2 ) {
  std::cout << "到终点了!" << std::endl; 
  break;}
  if (stop_and_restart == 1)//&&(std::pow((last_path.s.back() - final_path_show.s[0]), 2) <= 1))
  {
    std::cout << "需要重新规划!!" << std::endl;
    break;
  }
  }
  plt::pause(10000);
  //  const char* filename = "./basic.png";  //有问题
  // std::cout << "Saving result to " << filename << std::endl;;
  // plt::save(filename);
    return 0;
  }
