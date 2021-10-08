/*
 * @Author: wpbit
 * @Date: 2021-09-25 20:28:10
 * @LastEditTime: 2021-10-07 20:23:06
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
        nh, "/delphi_0/delphi_esr", 10, ros::TransportHints().tcpNoDelay());
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

geometry_msgs::Point RadarCalibration::polar_xy(const can_msgs::delphi_msg polar_xy_in)
{
    geometry_msgs::Point point_xy;
    point_xy.x = polar_xy_in.range * std::sin(polar_xy_in.angel*pi/180);
    point_xy.y = polar_xy_in.range * std::cos(polar_xy_in.angel*pi/180);
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

std::vector<can_msgs::delphi_msg> RadarCalibration::radar_filter(const std::vector<can_msgs::delphi_msg> delphi_in)
{
    std::vector<can_msgs::delphi_msg> radar_filter_output;
    for(int i = 0; i < delphi_in.size(); i++)
    {
        //滤波
        if(delphi_in[i].status == 0 || delphi_in[i].range > 15)
        {
            continue;
        }else{
            radar_filter_output.push_back(delphi_in[i]);
        }
    }
    return radar_filter_output;
}

std::vector<radarinfo> RadarCalibration::space_ok(const std::vector<can_msgs::delphi_msg> space_in_)
{
    //输出结果
    std::vector<radarinfo> radarinfo_out;
    for(int i = 0; i < space_in_.size(); i++)
    {
        radarinfo test_out;
        test_out.pxy = position_transform(polar_xy(space_in_[i]));
        test_out.prange = space_in_[i].range;
        test_out.pspeed = space_in_[i].rate;
        test_out.pdb = space_in_[i].db;
        //ROS_INFO("point%d: x=%d, y=%d", i, test_out.pxy.x, test_out.pxy.y);
        //ROS_INFO("point%d: db=%f", i, test_out.pdb);
        //滤除投影结果在图像之外的点
        if(test_out.pxy.x>=0 && test_out.pxy.x<=WIDTH && test_out.pxy.y>=0 && test_out.pxy.y<=HEIGHT)
        {
            radarinfo_out.push_back(test_out);
        }
    }
    return radarinfo_out;
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
    std::vector<can_msgs::delphi_msg> delphi_point;
    delphi_point = radar_filter(delphi_);
    std::vector<radarinfo> radar_point;
    radar_point = space_ok(delphi_point);
    ros::NodeHandle nh_;
    sub_param = nh_.subscribe("/dynamic_calibration/radar_transform_camera", 10, &RadarCalibration::paramcallback, this);
    //可视化
    if(!radar_point.empty())
    {
        for(int j = 0; j<radar_point.size(); j++)
        {
            //描点
            cv::circle(cv_ptr->image, 
                cv::Point(radar_point[j].pxy.x, radar_point[j].pxy.y), 8, CV_RGB(255,0,0), -1);
            //显示range和反射率
            char str[16];
            //sprintf(str, "%.2f %.2f", radar_point[j].prange, radar_point[j].pspeed);
            sprintf(str, "%.2f %.2f %.2f", radar_point[j].prange, radar_point[j].pspeed, radar_point[j].pdb);
            cv::putText(cv_ptr->image, str, cv::Point(radar_point[j].pxy.x, radar_point[j].pxy.y), 
                       cv::FONT_HERSHEY_TRIPLEX, 1, CV_RGB(255,0,0), 2);
        }
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("reveived image!");
}

