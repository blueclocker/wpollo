/*
 * @Author: wpbit
 * @Date: 2021-09-08 19:27:18
 * @LastEditTime: 2021-10-06 19:46:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/camera_radar/src/camera_radar.cpp
 */

#include "camera_radar/camera_radar.h"

CameraRadarCore::CameraRadarCore(ros::NodeHandle &nh_ws)
{
    //设置参数
    if(set_param())
    {
        ROS_INFO("param set finished!");
    }
    //定义发布器
    pub = nh_ws.advertise<sensor_msgs::Image>("fusion_image", 100);
    //时间融合
    sub_camera = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, "/galaxy_camera/image_raw", 1, ros::TransportHints().tcpNoDelay()
    );
    sub_bboxes = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(
        nh, "/darknet_ros/bounding_boxes", 1, ros::TransportHints().tcpNoDelay()
    );
    sub_radar  = new message_filters::Subscriber<can_msgs::delphi_msges>(
        nh, "/delphi_0/delphi_esr", 1, ros::TransportHints().tcpNoDelay()
    );
    two_sync = new message_filters::Synchronizer<two_syncPolicy>(
        two_syncPolicy(10), *sub_camera, *sub_radar
    );
    three_sync = new message_filters::Synchronizer<three_syncPolicy>(
        three_syncPolicy(10), *sub_camera, *sub_bboxes, *sub_radar
    );
    //回调函数
    three_sync->registerCallback(boost::bind(&CameraRadarCore::three_Callback, this, _1, _2, _3));
    two_sync->registerCallback(boost::bind(&CameraRadarCore::two_Callback, this, _1, _2));
    //ros多线程
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
}

CameraRadarCore::~CameraRadarCore()
{
    delete sub_camera;
    delete sub_bboxes;
    delete sub_radar;
    delete two_sync;
    delete three_sync;
}

bool CameraRadarCore::set_param()
{
    XmlRpc::XmlRpcValue camera_param, tf_param, tf_long_param;
    bool tf_flag = false, camera_flag = false, tf_long_flag = false;
    //世界坐标变换参数，平移x,y,z;旋转x,y,z(rad)
    Eigen::Matrix<double, 6, 1> tf_6d, tf_6d_long;
    //读取yaml tf_6d参数
    if(!nh.getParam("config_tf", tf_param))
    {
        ROS_ERROR("Failed to get tf parameter from server.");
    }else{
        for (int i = 0; i < tf_param.size(); i++)
        {
            tf_6d(i) = (double)tf_param[i];
            //ROS_INFO("tf_6d[%d]=%f", i, tf_6d(i));
        }
        tf_flag = true;
    }
    //tf_6d << 0, 0, 0, pi/2, pi, 0;
    //读取yaml tf_6d_long参数
    /*if(!nh.getParam("config_tf_long", tf_long_param))
    {
        ROS_ERROR("Failed to get tf_long parameter from server.");
    }else{
        for (int i = 0; i < tf_long_param.size(); i++)
        {
            tf_6d_long(i) = (double)tf_long_param[i];
            //ROS_INFO("tf_6d[%d]=%f", i, tf_6d(i));
        }
        tf_long_flag = true;
    }*/
    //读取yaml camera_martxia参数
    if(!nh.getParam("camera/config_camera", camera_param))
    {
        ROS_ERROR("Failed to get camera parameter from server.");
    }else{
        for (int i = 0; i < camera_param.size(); i++)
        {
            camera_matrix(i/3,i%3) = (double)camera_param[i];
            //ROS_INFO("camera[%d]=%f", i, camera_matrix(i/3,i%3));
        }
	    camera_flag = true;
    }
    //内参矩阵
    //Eigen::Matrix<double, 3, 3> camera_matrix;
    //camera_matrix << 4306.37205655706, 0, 802.1519317540232, 
    //                 0, 4307.411849731741, 479.2531644403244, 
    //                 0, 0, 1;
    //平移矩阵
    Eigen::Matrix<double, 4, 4> move;
    move << 1.0, 0.0, 0.0, tf_6d(0),
            0.0, 1.0, 0.0, tf_6d(1),
            0.0, 0.0, 1.0, tf_6d(2),
            0.0, 0.0, 0.0, 1.0;
    //x旋转矩阵
    double x_ = tf_6d(3);
    Eigen::Matrix<double, 4, 4> twist_x;
    twist_x << 1.0, 0.0, 0.0, 0.0,
               0.0, std::cos(x_), -std::sin(x_), 0.0,
               0.0, std::sin(x_), std::cos(x_), 0.0,
               0.0, 0.0, 0.0, 1.0;
    //y旋转矩阵
    double y_ = tf_6d(4);
    Eigen::Matrix<double, 4, 4> twist_y;
    twist_y << std::cos(y_), 0.0, std::sin(y_), 0.0,
               0.0, 1.0, 0.0, 0.0,
               -std::sin(y_), 0.0, std::cos(y_), 0.0,
               0.0, 0.0, 0.0, 1.0;
    //z旋转矩阵
    double z_ = tf_6d(5);
    Eigen::Matrix<double, 4, 4> twist_z;
    twist_z << std::cos(z_), -std::sin(z_), 0.0, 0.0,
               std::sin(z_), std::cos(z_), 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0;
    //变换矩阵,平移和旋转
    //Eigen::Matrix<double, 4, 4> trans_matrix;
    trans_matrix = move * twist_x * twist_y * twist_z;
    /*tf_long
    move << 1.0, 0.0, 0.0, tf_6d_long(0),
            0.0, 1.0, 0.0, tf_6d_long(1),
            0.0, 0.0, 1.0, tf_6d_long(2),
            0.0, 0.0, 0.0, 1.0;
    //x旋转矩阵
    x_ = tf_6d_long(3);
    twist_x << 1.0, 0.0, 0.0, 0.0,
               0.0, std::cos(x_), -std::sin(x_), 0.0,
               0.0, std::sin(x_), std::cos(x_), 0.0,
               0.0, 0.0, 0.0, 1.0;
    //y旋转矩阵
    y_ = tf_6d_long(4);
    twist_y << std::cos(y_), 0.0, std::sin(y_), 0.0,
               0.0, 1.0, 0.0, 0.0,
               -std::sin(y_), 0.0, std::cos(y_), 0.0,
               0.0, 0.0, 0.0, 1.0;
    //z旋转矩阵
    z_ = tf_6d_long(5);
    twist_z << std::cos(z_), -std::sin(z_), 0.0, 0.0,
               std::sin(z_), std::cos(z_), 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0;*/
    //变换矩阵,平移和旋转
    //Eigen::Matrix<double, 4, 4> trans_matrix;
    //trans_matrix_long = move * twist_x * twist_y * twist_z;
    if(tf_flag && camera_flag)
    {
        nh.param("camera/camera_width", WIDTH, 1920);
        nh.param("camera/camera_height", HEIGHT, 1200);
        nh.param("iou_key", IOU_KEY, 0.6);
        nh.param("car_width", CAR_WIDTH);
        nh.param("car_height", CAR_HEIGHT);
        return true;
    }else{
        return false;
    }
}

std::vector<can_msgs::delphi_msg> CameraRadarCore::radar_filter(const std::vector<can_msgs::delphi_msg> delphi_in)
{
    std::vector<can_msgs::delphi_msg> radar_filter_output;
    for(int i = 0; i < delphi_in.size(); i++)
    {
        if(delphi_in[i].status == 0 || delphi_in[i].range > 60)
        {
            continue;
        }else{
            radar_filter_output.push_back(delphi_in[i]);
        }
    }
    return radar_filter_output;
}

geometry_msgs::Point CameraRadarCore::polar_xy(const can_msgs::delphi_msg polar_xy_in)
{
    geometry_msgs::Point point_xy;
    point_xy.x = polar_xy_in.range * std::sin(polar_xy_in.angel*pi/180);
    point_xy.y = polar_xy_in.range * std::cos(polar_xy_in.angel*pi/180);
    point_xy.z = 0;
    return point_xy;
}

pixel_position CameraRadarCore::position_transform(const geometry_msgs::Point in_3d)
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

std::vector<radarinfo> CameraRadarCore::space_ok(const std::vector<can_msgs::delphi_msg> space_in_)
{
    //滤波
    std::vector<can_msgs::delphi_msg> filter_space_in = radar_filter(space_in_);
    //输出结果
    std::vector<radarinfo> radarinfo_out;
    for(int i = 0; i < filter_space_in.size(); i++)
    {
        radarinfo test_out;
        test_out.pxy = position_transform(polar_xy(filter_space_in[i]));
        test_out.prange = filter_space_in[i].range;
        test_out.pspeed = filter_space_in[i].rate;
        //ROS_INFO("point%d: x=%d, y=%d", i, test_out.x, test_out.y);
        //滤除投影结果在图像之外的点
        if(test_out.pxy.x>=0 && test_out.pxy.x<=WIDTH && test_out.pxy.y>=0 && test_out.pxy.y<=HEIGHT)
        {
            radarinfo_out.push_back(test_out);
        }
    }
    return radarinfo_out;
}

double CameraRadarCore::IOU(const cv::Rect &r1, const cv::Rect &r2)
{
    cv::Rect N = r1 | r2;
    cv::Rect U = r1 & r2;
    return U.area()*1.0 / N.area();
}

double CameraRadarCore::distance(const pixel_position pos_a, const pixel_position pos_b)
{
    return std::sqrt((pos_a.x - pos_b.x) * (pos_a.x - pos_b.x) +
                     (pos_a.y - pos_b.y) * (pos_a.y - pos_b.y));
}

void CameraRadarCore::knn_match(const std::vector<darknet_ros_msgs::BoundingBox> bbox_knn, 
                                const std::vector<radarinfo> radar_knn)
{
    ROS_INFO("matching ...");
    std::vector<pixel_position> bbox_knn_in;
    for(int i = 0; i < bbox_knn.size(); i++)
    {
        pixel_position temp;
        temp.x = (int)((bbox_knn[i].xmin + bbox_knn[i].xmax) / 2);
        temp.y = (int)((bbox_knn[i].ymin + bbox_knn[i].ymax) / 2);
        bbox_knn_in.push_back(temp);
    }
    std::vector<radarinfo> radar_knn_in = radar_knn;
    for(int j = 0; j < bbox_knn_in.size(); j++)
    {
        int radar_mark = 0;
        double dis = 100000;
        for(int k = 0; k < radar_knn_in.size(); k++)
        {
            if(radar_knn_in[k].pxy.x == -1){continue;}
            double dis_temp;
            dis_temp = distance(bbox_knn_in[j], radar_knn_in[k].pxy);
            if(dis_temp < dis)
            {
                dis = dis_temp;
                radar_mark = k;
            }
        }
        if(dis < 100)
        {
            //保存匹配关系
            matchmap.insert(std::pair<int, int>(j, radar_mark));
            //删去已经匹配的雷达点
            radar_knn_in[radar_mark].pxy = {-1, -1};
        }
    }
}

//ing,不能使用
void CameraRadarCore::iou_match(const std::vector<darknet_ros_msgs::BoundingBox> bbox_iou, 
                                const std::vector<radarinfo> radar_iou)
{
    std::vector<cv::Rect> bbox_iou_in, radar_iou_in;
    cv::Rect std_rect(-1, -1, -1, -1);
    for(int i = 0; i < bbox_iou.size(); i++)
    {
        cv::Rect bbox_cv(bbox_iou[i].xmin, bbox_iou[i].ymin, 
                         bbox_iou[i].xmax - bbox_iou[i].xmin,
                         bbox_iou[i].ymax - bbox_iou[i].ymin);
        bbox_iou_in.push_back(bbox_cv);
    }
    for(int i = 0; i < radar_iou.size(); i++)
    {
        //后续还需根据距离变换框的大小*********
        cv::Rect radar_cv(radar_iou[i].pxy.x - CAR_WIDTH/2, radar_iou[i].pxy.y - CAR_HEIGHT/2, 
                          CAR_WIDTH, CAR_HEIGHT);
        radar_iou_in.push_back(radar_cv);
    }
    for(int i = 0; i < bbox_iou_in.size(); i++)
    {
        int iou_mark = 0;
        double iou_ = 1.0;
        for(int j = 0; j < radar_iou_in.size(); j++)
        {
            if(radar_iou_in[j] == std_rect){continue;}
            double iou_temp;
            iou_temp = IOU(bbox_iou_in[i], radar_iou_in[j]);
            if(iou_temp < iou_)
            {
                iou_ = iou_temp;
                iou_mark = j;
            }
        }
        if(iou_ < IOU_KEY)
        {
            matchmap.insert(std::pair<int, int>(i, iou_mark));
            radar_iou_in[iou_mark] = {-1, -1, -1, -1};
        }
    }
}

//测试坐标变换回调函数
void CameraRadarCore::two_Callback(const sensor_msgs::Image::ConstPtr &two_img,
                                   const can_msgs::delphi_msges::ConstPtr &two_radar)
{
    /*std::vector<geometry_msgs::Point> space_in;
    //雷达输入二维极坐标
    std::vector<std::vector<double> > polar;
    std::vector<double> a_0 = {100, 1.57};
    polar.push_back(a_0);
    std::vector<double> a_1 = {100, 0.5};
    polar.push_back(a_1);
    std::vector<double> a_2 = {100, 3.14};
    polar.push_back(a_2);
    space_in = polar_xy(polar);
    //空间融合
    pos_filtered = space_ok(space_in);
    if(pos_filtered.size() != 0)
    {
        ROS_INFO("pos_filtered_size=%d", (int)pos_filtered.size());
    }else{
        ROS_INFO("NO POINT IN PIXEL COORDINATES!");
    }
    cv::Rect rect(50, 50, 200, 200);
    draw_picture(msg, rect);
    ROS_INFO("only camera!");*/
    if(!bbox_flag)
    {
        std::vector<can_msgs::delphi_msg> two_delphi_ = two_radar -> delphi_msges;
        //雷达无效点滤波与雷达信息转换radarinfo
        std::vector<radarinfo> two_radar_point;
        two_radar_point = space_ok(two_delphi_);
        //结果用OPENCV可视化
        draw_picture(two_img, two_radar_point);
        ROS_INFO("only radar!");
    }
    bbox_flag = false;
}

void CameraRadarCore::three_Callback(const sensor_msgs::Image::ConstPtr &three_camera, 
                    const darknet_ros_msgs::BoundingBoxes::ConstPtr &three_bboxes,
                    const can_msgs::delphi_msges::ConstPtr &three_radar)
{
    //ros_header = three_camera->header;
    //雷达输入二维极坐标
    std::vector<darknet_ros_msgs::BoundingBox> three_bboxes_ = three_bboxes -> bounding_boxes;
    std::vector<can_msgs::delphi_msg> three_delphi_ = three_radar -> delphi_msges;
    //雷达无效点滤波与雷达信息转换radarinfo
    std::vector<radarinfo> three_radar_point;
    three_radar_point = space_ok(three_delphi_);
    //匹配
    knn_match(three_bboxes_, three_radar_point);
    //结果用OPENCV可视化
    draw_picture(three_camera, three_bboxes_, three_radar_point);
    ROS_INFO("sensor fusion!");
    bbox_flag = true;
}

void CameraRadarCore::draw_picture(const sensor_msgs::Image::ConstPtr &msg_in, 
                const std::vector<darknet_ros_msgs::BoundingBox> &b_in,
                const std::vector<radarinfo> &delphi_point_in)
{
    //ros msg -> cv::mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
    //画检测框
    for(int j = 0; j < b_in.size(); j++)
    {
        cv::Rect rect_in(b_in[j].xmin, b_in[j].ymin, 
                         b_in[j].xmax-b_in[j].xmin, b_in[j].ymax-b_in[j].ymin);
        cv::putText(cv_ptr->image, b_in[j].Class.c_str(), cv::Point(b_in[j].xmin, b_in[j].ymin+40), 
                    cv::FONT_HERSHEY_TRIPLEX, 1.5, CV_RGB(255,0,0), 2); //cv::Scalar(b, g, r)
        cv::rectangle(cv_ptr->image, rect_in, CV_RGB(255,0,0), 4, cv::LINE_8, 0);
        //ROS_INFO("class:%s", b_in[j].Class.c_str());
    }
    if(!delphi_point_in.empty())
    {
        for(int j = 0; j<delphi_point_in.size(); j++)
        {
            cv::circle(cv_ptr->image, 
                cv::Point(delphi_point_in[j].pxy.x, delphi_point_in[j].pxy.y), 10, CV_RGB(255,0,0), 5);
        }
    }
    for(std::unordered_map<int,int>::iterator it = matchmap.begin(); it != matchmap.end(); it++)
    {
        ROS_INFO("yolo %dbbox matched!", it->first);
        cv::circle(cv_ptr->image, 
                cv::Point(delphi_point_in[it->second].pxy.x, delphi_point_in[it->second].pxy.y), 10, CV_RGB(255,255,255), 5);
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
}

void CameraRadarCore::draw_picture(const sensor_msgs::Image::ConstPtr &msg_in, 
                const std::vector<radarinfo> &delphi_point_in)
{
    //ros msg -> cv::mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
    if(!delphi_point_in.empty())
    {
        for(int j = 0; j<delphi_point_in.size(); j++)
        {
            cv::circle(cv_ptr->image, 
                cv::Point(delphi_point_in[j].pxy.x, delphi_point_in[j].pxy.y), 10, CV_RGB(255,0,0), 5);
        }
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
}
