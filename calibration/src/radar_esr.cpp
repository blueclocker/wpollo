/*
 * @Author: wpbit
 * @Date: 2021-09-25 20:28:10
 * @LastEditTime: 2021-09-29 21:27:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/calibration/src/radar_esr.cpp
 */
#include "calibration/radar_esr.h"

RadarCalibration::RadarCalibration(ros::NodeHandle &nh_ws)
{
    if(set_param(nh))
    {
        ROS_INFO("param set finished!");
    }
    pub = nh_ws.advertise<sensor_msgs::Image>("calibration_image", 100);
    sub_camera = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, "/galaxy_camera/image_raw", 10, ros::TransportHints().tcpNoDelay());
    sub_esr = new message_filters::Subscriber<can_msgs::delphi_msges>(
        nh, "/delphi_esr", 10, ros::TransportHints().tcpNoDelay());
    sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *sub_camera, *sub_esr);
    sync->registerCallback(boost::bind(&RadarCalibration::callback, this, _1, _2));
    ros::spin();
}

RadarCalibration::~RadarCalibration()
{
    delete sub_camera;
    delete sub_esr;
    delete sync;
}

bool RadarCalibration::set_param(ros::NodeHandle &n)
{
    XmlRpc::XmlRpcValue camera_param;
    bool camera_flag = false;
    if(!n.getParam("camera/config_camera", camera_param))
    {
        ROS_ERROR("Failed to get camera parameter from server.");
    }else{
        for (int i = 0; i < camera_param.size(); i++)
        {
            camera_matrix(i/3,i%3) = (double)camera_param[i];
        }
        n.param("camera/camera_width", WIDTH, 1920);
        n.param("camera/camera_height", HEIGHT, 1200);
        transtocamera.linear.x = 0;
        transtocamera.linear.y = 0;
        transtocamera.linear.z = 0;
        transtocamera.angular.x = 0;
        transtocamera.angular.y = 0;
        transtocamera.angular.z = 0;
        trans_matrix << -1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0;
        camera_flag = true;
    }
    return camera_flag;
}

geometry_msgs::Point RadarCalibration::polar_xy(const float range_, const float angle_)
{
    geometry_msgs::Point point_xy;
    point_xy.x = range_ * std::sin(angle_*pi/180);
    point_xy.y = range_ * std::cos(angle_*pi/180);
    point_xy.z = 0;
    return point_xy;
}

pixel_position RadarCalibration::position_transform(const geometry_msgs::Point in_3d)
{
    //四维齐次坐标
    Eigen::Vector4d world_pos_4d(in_3d.x, in_3d.y, in_3d.z, 1);
    //相机和雷达三维坐标系变换
    Eigen::Vector4d world_pos_4d_;
    world_pos_4d_ = trans_matrix * world_pos_4d;
    //3维世界坐标
    Eigen::Vector3d world_pos;
    world_pos << world_pos_4d_(0), world_pos_4d_(1), world_pos_4d_(2);
    //2维像素坐标
    Eigen::Vector3d pixel_pos;
    //世界坐标转像素坐标
    pixel_pos = camera_matrix * world_pos / world_pos_4d_(2);
    //保存像素坐标
    pixel_position out_2d;
    out_2d.x = (int)pixel_pos(0);
    out_2d.y = (int)pixel_pos(1);
    return out_2d;
}

std::vector<geometry_msgs::Point> RadarCalibration::radar_filter(const std::vector<can_msgs::delphi_msg> delphi_in)
{
    std::vector<geometry_msgs::Point> radar_filter_output;
    for(int i = 0; i < delphi_in.size(); i++)
    {
        //滤波
        if(delphi_in[i].status == 0 || delphi_in[i].range > 60)
        {
            continue;
        }else{
            radar_filter_output.push_back(polar_xy(delphi_in[i].range, delphi_in[i].angel));
        }
    }
    return radar_filter_output;
}

std::vector<pixel_position> RadarCalibration::space_ok(const std::vector<geometry_msgs::Point> space_in_)
{
    //输出结果
    std::vector<pixel_position> pixel_out;
    for(int i = 0; i < space_in_.size(); i++)
    {
        pixel_position test_out;
        test_out = position_transform(space_in_[i]);
        ROS_INFO("point%d: x=%d, y=%d", i, test_out.x, test_out.y);
        //滤除投影结果在图像之外的点
        if(test_out.x>=0 && test_out.x<=WIDTH && test_out.y>=0 && test_out.y<=HEIGHT)
        {
            pixel_out.push_back(test_out);
        }
    }
    return pixel_out;
}

void RadarCalibration::paramcallback(const geometry_msgs::Twist::ConstPtr &param)
{
    transtocamera.linear.x = param -> linear.x;
    transtocamera.linear.y = param -> linear.y;
    transtocamera.linear.z = param -> linear.z;
    transtocamera.angular.x = param -> angular.x;
    transtocamera.angular.y = param -> angular.y;
    transtocamera.angular.z = param -> angular.z;
    Eigen::Matrix<double, 4, 4> move;
    move << 1.0, 0.0, 0.0, transtocamera.linear.x,
            0.0, 1.0, 0.0, transtocamera.linear.y,
            0.0, 0.0, 1.0, transtocamera.linear.z,
            0.0, 0.0, 0.0, 1.0;
    //x旋转矩阵
    double x_ = transtocamera.angular.x;
    Eigen::Matrix<double, 4, 4> twist_x;
    twist_x << 1.0, 0.0, 0.0, 0.0,
               0.0, std::cos(x_), -std::sin(x_), 0.0,
               0.0, std::sin(x_), std::cos(x_), 0.0,
               0.0, 0.0, 0.0, 1.0;
    //y旋转矩阵
    double y_ = transtocamera.angular.y;
    Eigen::Matrix<double, 4, 4> twist_y;
    twist_y << std::cos(y_), 0.0, std::sin(y_), 0.0,
               0.0, 1.0, 0.0, 0.0,
               -std::sin(y_), 0.0, std::cos(y_), 0.0,
               0.0, 0.0, 0.0, 1.0;
    //z旋转矩阵
    double z_ = transtocamera.angular.z;
    Eigen::Matrix<double, 4, 4> twist_z;
    twist_z << std::cos(z_), -std::sin(z_), 0.0, 0.0,
               std::sin(z_), std::cos(z_), 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0;
    //变换矩阵,平移和旋转
    trans_matrix = move * twist_x * twist_y * twist_z;
    ROS_INFO("param update!");
}

void RadarCalibration::callback(const sensor_msgs::Image::ConstPtr &msg, 
              const can_msgs::delphi_msges::ConstPtr &msg_esr)
{
    //ros msg -> cv::mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    std::vector<can_msgs::delphi_msg> delphi_ = msg_esr -> delphi_msges;
    std::vector<geometry_msgs::Point> delphi_point;
    delphi_point = radar_filter(delphi_);
    std::vector<pixel_position> pixel_point;
    pixel_point = space_ok(delphi_point);
    ros::NodeHandle nh_;
    sub_param = nh_.subscribe("/dynamic_calibration/radar_transform_camera", 10, &RadarCalibration::paramcallback, this);
    //可视化
    if(!pixel_point.empty())
    {
        for(int j = 0; j<pixel_point.size(); j++)
        {
            cv::circle(cv_ptr->image, 
                cv::Point(pixel_point[j].x, pixel_point[j].y), 10, CV_RGB(255,0,0), 5);
        }
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("reveived image!");
}

