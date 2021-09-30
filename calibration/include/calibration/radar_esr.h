/*
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│
 *  ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\ │`~ ││
 *  │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│
 *  ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││
 *  │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│
 *  ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  ││
 *  │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│
 *  ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││
 *  │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│
 *  │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │
 *  │      └───┴─────┴───────────────────────┴─────┴───┘          │
 *  └─────────────────────────────────────────────────────────────┘
 *
 *
 * @Author: wpbit
 * @Date: 2021-09-26 14:30:44
 * @LastEditTime: 2021-09-26 15:36:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/calibration/include/calibration/radar_esr.h
 */
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <vector>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <can_msgs/delphi_msg.h>
#include <can_msgs/delphi_msges.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
//时间同步
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h> 

#define pi 3.1415926
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image,can_msgs::delphi_msges>syncPolicy;

//自定义单个雷达点像素坐标结构
typedef struct Pixel_position
{
        //x
        int x;
        //y
        int y;
}pixel_position;

class RadarCalibration
{
    private:
        //相机宽、高
        int WIDTH, HEIGHT;
        //内参矩阵
        Eigen::Matrix<double, 3, 3> camera_matrix;
        //外参矩阵
        Eigen::Matrix<double, 4, 4> trans_matrix;
        //雷达相机变换矩阵
        geometry_msgs::Twist transtocamera;
        ros::NodeHandle nh;
        //结果发布器
        ros::Publisher pub;
        //订阅动态参数
        ros::Subscriber sub_param;
        //相机、雷达时间同步
        message_filters::Subscriber<sensor_msgs::Image> *sub_camera;
        message_filters::Subscriber<can_msgs::delphi_msges> *sub_esr;
        message_filters::Synchronizer<syncPolicy> *sync;
        //参数初始化，从yaml加载相机内参，初始化transtocamera为0
        bool set_param(ros::NodeHandle &n);
        //雷达点极坐标转直角坐标
        geometry_msgs::Point polar_xy(const float range_, const float angle_);
        //单个雷达点投影到像素坐标系
        pixel_position position_transform(const geometry_msgs::Point in_3d);
        //滤除雷达噪点，并调用polar_xy
        std::vector<geometry_msgs::Point> radar_filter(const std::vector<can_msgs::delphi_msg> delphi_in);
        //多个雷达点投影到像素坐标系，并去除在图像之外点
        std::vector<pixel_position> space_ok(const std::vector<geometry_msgs::Point> space_in_);
        //更新transtocamera和trans_matrix
        void paramcallback(const geometry_msgs::Twist::ConstPtr &param);
        //相机和雷达数据融合
        void callback(const sensor_msgs::Image::ConstPtr &msg, 
                      const can_msgs::delphi_msges::ConstPtr &msg_esr);

    public:
        RadarCalibration(ros::NodeHandle &nh);
        ~RadarCalibration();
};
