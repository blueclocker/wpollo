/*
 * @Author: wpbit
 * @Date: 2021-09-08 19:27:18
 * @LastEditTime: 2021-10-31 22:44:41
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/camera_radar/src/camera_radar.cpp
 */

#include "camera_radar/camera_radar.h"
#include "camera_radar/Hungarian.h"

CameraRadarCore::CameraRadarCore(ros::NodeHandle &nh_ws)
{
    //设置参数
    if(set_param(nh_ws))
    {
        ROS_INFO("param set finished!");
    }
    //ROS_INFO("car_width = %d", CAR_WIDTH);
    //ROS_INFO("anchor scales = %.2f", ratios[1]);
    //定义发布器
    pub = nh_ws.advertise<sensor_msgs::Image>(camera_radarTopicName, camera_radarTopicqueuesize);
    //发布雷达bbox
    pub_mark = nh_ws.advertise<visualization_msgs::MarkerArray>("delphi_marker", 10); 
    //时间融合
    sub_camera = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, cameraTopicName, cameraTopicqueuesize, ros::TransportHints().tcpNoDelay()
    );
    sub_bboxes = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(
        nh, detectionTopicName, detectionTopicqueuesize, ros::TransportHints().tcpNoDelay()
    );
    sub_radar  = new message_filters::Subscriber<can_msgs::delphi_msges>(
        nh, radarTopicName, radarTopicqueuesize, ros::TransportHints().tcpNoDelay()
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

bool CameraRadarCore::set_param(ros::NodeHandle &nh_param)
{
    XmlRpc::XmlRpcValue camera_param, tf_param, scales_, ratios_;
    bool tf_flag = false, camera_flag = false, anchor_flag = false;
    //读取anchor参数
    if(!nh_param.getParam("match/anchor/scales", scales_))
    {
        ROS_ERROR("Failed to get anchor scales from server.");
    }else{
        scales.resize(scales_.size());
        for(int i = 0; i < scales_.size(); i++)
        {
            scales[i] = int(scales_[i]);
        }
        anchor_flag = true;
    }
    if(!nh_param.getParam("match/anchor/ratios", ratios_))
    {
        ROS_ERROR("Failed to get anchor ratios from server.");
    }else{
        ratios.resize(ratios_.size());
        for(int i = 0; i < ratios_.size(); i++)
        {
            ratios[i] = double(ratios_[i]);
        }
        anchor_flag = true;
    }
    //世界坐标变换参数，平移x,y,z;旋转x,y,z(rad)
    Eigen::Matrix<double, 6, 1> tf_6d;
    //读取yaml tf_6d参数
    if(!nh_param.getParam("camera/config_tf", tf_param))
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
    //读取yaml camera_martxia参数
    if(!nh_param.getParam("camera/config_camera", camera_param))
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
    if(tf_flag && camera_flag && anchor_flag)
    {
        nh_param.param<int>("camera/camera_width", WIDTH, 1920);
        nh_param.param<int>("camera/camera_height", HEIGHT, 1200);
        nh_param.param<double>("match/iou_key", IOU_KEY, 0.6);
        nh_param.param<std::string>("subscribers/camera_reading/topic", cameraTopicName, std::string("/galaxy_camera/image_raw"));
        nh_param.param<int>("subscribers/camera_reading/queue_size", cameraTopicqueuesize, 1);
        nh_param.param<std::string>("subscribers/radar_reading/topic", radarTopicName, std::string("/delphi_0/delphi_esr"));
        nh_param.param<int>("subscribers/radar_reading/queue_size", radarTopicqueuesize, 1);
        nh_param.param<std::string>("subscribers/detection/topic", detectionTopicName, std::string("/darknet_ros/bounding_boxes"));
        nh_param.param<int>("subscribers/detection/queue_size", detectionTopicqueuesize, 1);
        nh_param.param<std::string>("publishers/camera_radar/topic", camera_radarTopicName, std::string("fusion_image"));
        nh_param.param<int>("publishers/camera_radar/queue_size", camera_radarTopicqueuesize, 100);
        return true;
    }else{
        return false;
    }
}

std::vector<cv::Rect> CameraRadarCore::anchor_generate(const pixel_position anchor_in)
{
    std::vector<cv::Rect> anchor_out;
    for(int i = 0; i < scales.size(); i++)
    {
        for(int j = 0; j < ratios.size(); j++)
        {
            int anchor_h, anchor_w;
            anchor_w = (int)std::sqrt(scales[i] * scales[i] * ratios[j]);
            anchor_h = (int)anchor_w / ratios[j];
            int left_a, top_a, right_a, bot_a;
            left_a = anchor_in.x - anchor_w/2;
            right_a = anchor_in.x + anchor_w/2;
            top_a = anchor_in.y - anchor_h/2;
            bot_a = anchor_in.y + anchor_h/2;
            if(left_a < 0) left_a = 0;
            if(right_a > WIDTH - 1) right_a = WIDTH -1;
            if(top_a < 0) top_a = 0;
            if(bot_a > HEIGHT - 1) bot_a = HEIGHT - 1;
            cv::Rect rect_single(left_a, top_a, right_a - left_a, bot_a - top_a);
            anchor_out.push_back(rect_single);
        }
    }
    return anchor_out;
}

std::vector<cv::Rect> CameraRadarCore::anchor_generate(const pixel_position anchor_in, const darknet_ros_msgs::BoundingBox &bbox_anchor)
{
    std::vector<cv::Rect> anchor_out;
    double ratios_ = (1.0*(bbox_anchor.xmax-bbox_anchor.xmin)/(bbox_anchor.ymax-bbox_anchor.ymin));
    //ROS_INFO("ratios=%.2f", ratios_);
    for(int i = 0; i < scales.size(); i++)
    {
        int anchor_h, anchor_w;
        anchor_w = (int)std::sqrt(scales[i] * scales[i] * ratios_);
        anchor_h = (int)anchor_w / ratios_;
        int left_a, top_a, right_a, bot_a;
        left_a = anchor_in.x - anchor_w/2;
        right_a = anchor_in.x + anchor_w/2;
        top_a = anchor_in.y - anchor_h/2;
        bot_a = anchor_in.y + anchor_h/2;
        if(left_a < 0) left_a = 0;
        if(right_a > WIDTH - 1) right_a = WIDTH -1;
        if(top_a < 0) top_a = 0;
        if(bot_a > HEIGHT - 1) bot_a = HEIGHT - 1;
        cv::Rect rect_single(left_a, top_a, right_a - left_a, bot_a - top_a);
        anchor_out.push_back(rect_single);
    }
    return anchor_out;
}

std::vector<can_msgs::delphi_msg> CameraRadarCore::radar_filter(const std::vector<can_msgs::delphi_msg> *delphi_in)
{
    std::vector<can_msgs::delphi_msg> radar_filter_output;
    for(int i = 0; i < delphi_in->size(); i++)
    {
        //ROS_INFO("--filter--");
        //滤波
        if(delphi_in->at(i).range < 1 || delphi_in->at(i).range > 120)
        {
            continue;
        }else{
            radar_filter_output.push_back(delphi_in->at(i));
        }
    }
    //ROS_INFO("end filter");
    return radar_filter_output;
}

geometry_msgs::Point CameraRadarCore::polar_xy(const can_msgs::delphi_msg &polar_xy_in)
{
    geometry_msgs::Point point_xy;
    point_xy.x = polar_xy_in.range * std::sin(polar_xy_in.angel*pi/180);
    point_xy.y = polar_xy_in.range * std::cos(polar_xy_in.angel*pi/180);
    point_xy.z = 0;
    return point_xy;
}

pixel_position CameraRadarCore::position_transform(const geometry_msgs::Point &in_3d)
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

std::vector<radarinfo> CameraRadarCore::space_ok(const std::vector<can_msgs::delphi_msg> *space_in_)
{
    //滤波
    std::vector<can_msgs::delphi_msg> filter_space_in = radar_filter(space_in_);
    //输出结果
    std::vector<radarinfo> radarinfo_out;
    for(int i = 0; i < filter_space_in.size(); i++)
    {
        radarinfo test_out;
        test_out.pxy = position_transform(polar_xy(filter_space_in[i]));
        //滤除投影结果在图像之外的点
        if(test_out.pxy.x>=0 && test_out.pxy.x<=WIDTH && test_out.pxy.y>=0 && test_out.pxy.y<=HEIGHT)
        {
            test_out.prange = filter_space_in[i].range;
            test_out.pspeed = filter_space_in[i].rate;
            test_out.ppoint = polar_xy(filter_space_in[i]);
            //test_out.pdb = filter_space_in[i].db;
            //ROS_INFO("point%d: x=%d, y=%d", i, test_out.x, test_out.y);
            radarinfo_out.push_back(test_out);
        }
    }
    return radarinfo_out;
}

double CameraRadarCore::measurement_width(const double &range_mea, const darknet_ros_msgs::BoundingBox &bbox_mea)
{
    double f = 25.0;//焦距25mm
    double p = 5.86 * 0.001;//像素尺寸mm/pixel
    return p * (bbox_mea.xmax - bbox_mea.xmin) / f * range_mea;
}

double CameraRadarCore::IOU(const cv::Rect &r1, const cv::Rect &r2)
{
    cv::Rect N = r1 | r2;
    cv::Rect U = r1 & r2;
    return U.area()*1.0 / N.area();
}

double CameraRadarCore::distance(const pixel_position &pos_a, const pixel_position &pos_b)
{
    return std::sqrt((pos_a.x - pos_b.x) * (pos_a.x - pos_b.x) +
                     (pos_a.y - pos_b.y) * (pos_a.y - pos_b.y));
}

double CameraRadarCore::threed_distance(const darknet_ros_msgs::BoundingBox &threed_distance_in)
{
    double f = 25.0;//焦距25mm
    double p = 5.86 * 0.001;//像素尺寸mm/pixel
    double objectWidth;
    if(threed_distance_in.Class == "car")
    {
        objectWidth = 1.7;
    }else if(threed_distance_in.Class == "person"){
        objectWidth = 0.5;
    }else if(threed_distance_in.Class == "bus" || threed_distance_in.Class == "truck"){
        objectWidth = 2.2;
    }else if(threed_distance_in.Class == "bicycle"){
        objectWidth = 0.3;
    }else{
        objectWidth = 2.0;
    }
    //单位:m
    return f * objectWidth / (p * (threed_distance_in.xmax-threed_distance_in.xmin));
}

void CameraRadarCore::knn_match(const std::vector<darknet_ros_msgs::BoundingBox> *bbox_knn, 
                                const std::vector<radarinfo> *radar_knn)
{
    ROS_INFO("knn matching ...");
    std::vector<pixel_position> bbox_knn_in;
    for(int i = 0; i < bbox_knn->size(); i++)
    {
        pixel_position temp;
        temp.x = (int)((bbox_knn->at(i).xmin + bbox_knn->at(i).xmax) / 2);
        temp.y = (int)((bbox_knn->at(i).ymin + bbox_knn->at(i).ymax) / 2);
        bbox_knn_in.push_back(temp);
    }
    std::vector<radarinfo> radar_knn_in = *radar_knn;
    for(int j = 0; j < bbox_knn_in.size(); j++)
    {
        int radar_mark = 0;
        double dis = DBL_MAX;
        for(int k = 0; k < radar_knn_in.size(); k++)
        {
            if(radar_knn_in[k].pxy.x == -1){continue;}
            double dis_temp;
            dis_temp = distance(bbox_knn_in[j], radar_knn_in[k].pxy)/(bbox_knn->at(j).xmax-bbox_knn->at(j).xmin);
            if(dis_temp < dis)
            {
                dis = dis_temp;
                radar_mark = k;
            }
        }
        if(dis < 1)
        {
            //保存匹配关系
            matchmap.insert(std::pair<int, int>(j, radar_mark));
            //删去已经匹配的雷达点
            radar_knn_in[radar_mark].pxy = {-1, -1};
        }else{
            break;
        }
    }
}

//使用一组锚框用作雷达检测框
void CameraRadarCore::iou_match(const std::vector<darknet_ros_msgs::BoundingBox> *bbox_iou, 
                                const std::vector<radarinfo> *radar_iou)
{
    ROS_INFO("iou matching ...");
    std::vector<cv::Rect> bbox_iou_in;
    std::vector<std::vector<cv::Rect> > radar_iou_in;
    //cv::Rect std_rect(-1, -1, -1, -1);
    bool radar_match_flag[radar_iou->size()] = {0}; 
    for(int i = 0; i < bbox_iou->size(); i++)
    {
        cv::Rect bbox_cv(bbox_iou->at(i).xmin, bbox_iou->at(i).ymin, 
                         bbox_iou->at(i).xmax - bbox_iou->at(i).xmin,
                         bbox_iou->at(i).ymax - bbox_iou->at(i).ymin);
        bbox_iou_in.push_back(bbox_cv);
        //ROS_INFO("bbox_area = %f", (double)bbox_cv.area());
    }
    for(int i = 0; i < radar_iou->size(); i++)
    {
        std::vector<cv::Rect> radar_cv;
        radar_cv = anchor_generate(radar_iou->at(i).pxy);
        radar_iou_in.push_back(radar_cv);
        //ROS_INFO("radar_x = %f", CAR_WIDTH);
        //ROS_INFO("radar_area = %f", (double)radar_cv.area());
    }
    /*for(int i = 0; i < bbox_iou_in.size(); i++)
    {
        int iou_mark = 0;
        double iou_ = 0;
        //对yolo某一检测框循计算与未匹配的雷达框的iou，取最小
        for(int j = 0; j < radar_iou_in.size(); j++)
        {
            if(radar_iou_in[j] == std_rect){continue;}
            double iou_temp;
            iou_temp = IOU(bbox_iou_in[i], radar_iou_in[j]);
            ROS_INFO("iou = %f", iou_temp);
            if(iou_temp > iou_)
            {
                iou_ = iou_temp;
                iou_mark = j;
            }
        }
        if(iou_ > IOU_KEY)
        {
            matchmap.insert(std::pair<int, int>(i, iou_mark));
            radar_iou_in[iou_mark] = {-1, -1, -1, -1};
            ROS_INFO("iou = %f", iou_);
        }
    }*/
    //锚框组匹配
    for(int i = 0; i < bbox_iou_in.size(); i++)
    {
        int iou_mark = 0;
        double iou_ = 0;
        cv::Rect rect_out;
        for(int j = 0; j < radar_iou_in.size(); j++)
        {
            if(radar_match_flag[j]) continue;
            for(int k = 0; k < radar_iou_in[j].size(); k++)
            {
                double iou_temp;
                iou_temp = IOU(bbox_iou_in[i], radar_iou_in[j][k]);
                //ROS_INFO("iou = %f", iou_temp);
                if(iou_temp > iou_)
                {
                    iou_ = iou_temp;
                    rect_out = radar_iou_in[j][k];
                    iou_mark = j;
                }
            }
        }
        if(iou_ > IOU_KEY)
        {
            matchmap.insert(std::pair<int, int>(i, iou_mark));
            ioumap.insert(std::pair<int, cv::Rect>(i, rect_out));
            radar_match_flag[iou_mark] = true;
            ROS_INFO("iou = %f", iou_);
        }
    }
}

//匈牙利算法多用于帧间匹配，在此用于目标关联勉强
void CameraRadarCore::hung_match(const std::vector<darknet_ros_msgs::BoundingBox> *bbox_hung, const std::vector<radarinfo> *radar_hung)
{
    ROS_INFO("hung matching ...");
    std::vector<pixel_position> bbox_hung_in;
    for(int i = 0; i < bbox_hung->size(); i++)
    {
        pixel_position temp;
        temp.x = (int)((bbox_hung->at(i).xmin + bbox_hung->at(i).xmax) / 2);
        temp.y = (int)((bbox_hung->at(i).ymin + bbox_hung->at(i).ymax) / 2);
        bbox_hung_in.push_back(temp);
    }
    std::vector<radarinfo> radar_hung_in = *radar_hung;
    std::vector<std::vector<double> > costMatrix;
    for(int i = 0; i < bbox_hung_in.size(); i++)
    {
        std::vector<double> costmatrix_line;
        for(int j = 0; j < radar_hung_in.size(); j++)
        {
            costmatrix_line.push_back(distance(bbox_hung_in[i], radar_hung_in[j].pxy));
        }
        costMatrix.push_back(costmatrix_line);
    }
    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;
    double cost = HungAlgo.Solve(costMatrix, assignment);
    for (int x = 0; x < costMatrix.size(); x++)
    {
        if(assignment[x] != -1)
        {
            if(costMatrix[x][assignment[x]] <= 100)
            {
                matchmap.insert(std::pair<int, int>(x, assignment[x]));
            }else{
                cost -= costMatrix[x][assignment[x]];
            }
        }
    }
    ROS_INFO("cost = %f", cost);
}

//融合iou匹配和knn匹配
void CameraRadarCore::total_match(const std::vector<darknet_ros_msgs::BoundingBox> *bbox_total, const std::vector<radarinfo> *radar_total)
{
    Eigen::MatrixXd dismatrix;
    dismatrix.resize(bbox_total->size(), radar_total->size());
    for(int i = 0; i < bbox_total->size(); i++)
    {
        double costmatch = threed_distance(bbox_total->at(i));
        for(int j = 0; j < radar_total->size(); j++)
        {
            pixel_position bbox_pixel_postion = {(int)(bbox_total->at(i).xmax+bbox_total->at(i).xmin)/2,
                                                 (int)(bbox_total->at(i).ymax+bbox_total->at(i).ymin)/2};
            double lamda = 2*distance(bbox_pixel_postion,radar_total->at(j).pxy)/(bbox_total->at(i).xmax-bbox_total->at(i).xmin+
                                                                            bbox_total->at(i).ymax-bbox_total->at(i).ymin);
            dismatrix(i, j) = std::fabs(costmatch - radar_total->at(j).prange) * lamda;
        }
    }
    for(int i = 0; i < bbox_total->size(); i++)
    {
        double iou_second = 0;
        int iou_mark;
        cv::Rect iou_rect;
        cv::Rect camera_match_bbox(bbox_total->at(i).xmin , bbox_total->at(i).ymin, 
                                   bbox_total->at(i).xmax - bbox_total->at(i).xmin,
                                   bbox_total->at(i).ymax - bbox_total->at(i).ymin);
        for(int j = 0; j < 3; j++)//至多3个预匹配点
        {
            int mincol;
            if(dismatrix.row(i).minCoeff(&mincol) < 10.0)
            {
                dismatrix(i, mincol) = DBL_MAX;
                std::vector<cv::Rect> radar_match_bbox = anchor_generate(radar_total->at(mincol).pxy, bbox_total->at(i));
                for(int k = 0; k < radar_match_bbox.size(); k++)
                {
                    double iou_first;
                    iou_first = IOU(camera_match_bbox, radar_match_bbox[k]);
                    if(iou_first > iou_second)
                    {
                        iou_second = iou_first;
                        iou_mark = mincol;
                        iou_rect = radar_match_bbox[k];
                    }
                }
            }else{
                break;
            }
        }
        if(iou_second > IOU_KEY)
        {
            matchmap.insert(std::pair<int, int>(i, iou_mark));
            ioumap.insert(std::pair<int, cv::Rect>(i, iou_rect));
            ROS_INFO("%s, iou=%.2f", bbox_total->at(i).Class.c_str(), iou_second);
        }
    }
}

visualization_msgs::Marker CameraRadarCore::pub_markerarray(const int id_pub ,const radarinfo &delphi_pub, const std::string &text_pub)
{
    //publish markerarray
    visualization_msgs::Marker msg_marker;
    msg_marker.header = ros_header;
    msg_marker.header.frame_id = "delphi_markerarray";
    msg_marker.id = id_pub;
    //msg_marker.type = visualization_msgs::Marker::CUBE;
    msg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg_marker.action = visualization_msgs::Marker::ADD;
    msg_marker.pose.position = delphi_pub.ppoint;
    msg_marker.pose.orientation.x = 0.0;
    msg_marker.pose.orientation.y = 0.0;
    msg_marker.pose.orientation.z = 0.0;
    msg_marker.pose.orientation.w = 1.0;
    //msg_marker.scale.x = 2.0;//width,m
    //msg_marker.scale.y = 4.0;//length,m
    msg_marker.scale.z = 0.5;//height,m
    msg_marker.color.r = 0.0;
    msg_marker.color.g = 1.0;
    msg_marker.color.b = 0.0;
    msg_marker.color.a = 0.5;
    msg_marker.text = text_pub;
    msg_marker.lifetime = ros::Duration(0);
    return msg_marker;
}

//无yolo检测信息回调函数
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
        two_radar_point = space_ok(&two_delphi_);
        //结果用OPENCV可视化
        draw_picture(two_img, two_radar_point);
        ROS_INFO("only radar!");
    }
    bbox_flag = false;
}

//完整回调函数
void CameraRadarCore::three_Callback(const sensor_msgs::Image::ConstPtr &three_camera, 
                    const darknet_ros_msgs::BoundingBoxes::ConstPtr &three_bboxes,
                    const can_msgs::delphi_msges::ConstPtr &three_radar)
{
    ros_header = three_camera->header;
    //雷达输入二维极坐标
    std::vector<darknet_ros_msgs::BoundingBox> three_bboxes_ = three_bboxes -> bounding_boxes;
    std::vector<can_msgs::delphi_msg> three_delphi_ = three_radar -> delphi_msges;
    //雷达无效点滤波与雷达信息转换radarinfo
    std::vector<radarinfo> three_radar_point;
    three_radar_point = space_ok(&three_delphi_);
    //匹配
    total_match(&three_bboxes_, &three_radar_point);
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
    //画检测框 first generation
    /*for(int j = 0; j < b_in.size(); j++)
    {
        cv::Rect rect_in(b_in[j].xmin, b_in[j].ymin, 
                         b_in[j].xmax-b_in[j].xmin, b_in[j].ymax-b_in[j].ymin);
        cv::putText(cv_ptr->image, b_in[j].Class.c_str(), cv::Point(b_in[j].xmin, b_in[j].ymin+40), 
                    cv::FONT_HERSHEY_TRIPLEX, 1, CV_RGB(255,0,0), 2); //cv::Scalar(b, g, r)
        cv::rectangle(cv_ptr->image, rect_in, CV_RGB(255,0,0), 4, cv::LINE_8, 0);
        //ROS_INFO("class:%s", b_in[j].Class.c_str());
    }
    if(!delphi_point_in.empty())
    {
        for(int j = 0; j<delphi_point_in.size(); j++)
        {
            cv::circle(cv_ptr->image, 
                cv::Point(delphi_point_in[j].pxy.x, delphi_point_in[j].pxy.y), 8, CV_RGB(255,0,0), -1);
        }
    }
    if(!matchmap.empty())
    {
        for(std::unordered_map<int,int>::iterator it = matchmap.begin(); it != matchmap.end(); it++)
        {
            ROS_INFO("yolo %dbbox matched!", it->first);
            cv::circle(cv_ptr->image, 
                cv::Point(delphi_point_in[it->second].pxy.x, delphi_point_in[it->second].pxy.y), 10, CV_RGB(255,255,255), 5);
        }
        matchmap.clear();
    }*/
    //画检测框 second generation
    //显示匹配成功结果
    bool draw_bbox_flag[b_in.size()] = {0};
    bool draw_radar_flag[delphi_point_in.size()] = {0};
    if(!matchmap.empty())
    {
        int id_count = 0;
        msg_marks.clear();
        for(std::unordered_map<int,int>::iterator it = matchmap.begin(); it != matchmap.end(); it++)
        {
            //ROS_INFO("yolo %dbbox matched!", it->first);
            //if(b_in[it->first].Class == "car")
            //{
            //雷达点
            cv::circle(cv_ptr->image, cv::Point(delphi_point_in[it->second].pxy.x, 
                        delphi_point_in[it->second].pxy.y), 8, CV_RGB(0,0,255), -1);
            cv::Rect rect_in(b_in[it->first].xmin, b_in[it->first].ymin, 
                             b_in[it->first].xmax-b_in[it->first].xmin, b_in[it->first].ymax-b_in[it->first].ymin);
            char str[16];
            //标签
            //sprintf(str, "%s %.2f", b_in[it->first].Class.c_str(), delphi_point_in[it->second].prange);
            //sprintf(str, "%s %.2f %.2f", b_in[it->first].Class.c_str(), 
            //       measurement_width(delphi_point_in[it->second].prange, b_in[it->first]), delphi_point_in[it->second].prange);
            sprintf(str, "%s %.2f %.2f", b_in[it->first].Class.c_str(), 
                    threed_distance(b_in[it->first]), delphi_point_in[it->second].prange);
            cv::putText(cv_ptr->image, str, cv::Point(b_in[it->first].xmin, b_in[it->first].ymin+40), 
                        cv::FONT_HERSHEY_TRIPLEX, 1, CV_RGB(255,0,0), 2); //cv::Scalar(b, g, r)
            //视觉检测框
            cv::rectangle(cv_ptr->image, rect_in, CV_RGB(255,0,0), 4, cv::LINE_8, 0);
            //雷达框 蓝色
            //cv::rectangle(cv_ptr->image, cv::Rect(ioumap[it->first]), CV_RGB(0,0,255), 4, cv::LINE_8, 0);
            draw_bbox_flag[it->first] = true;
            draw_radar_flag[it->second] = true;
            //std::vector<cv::Rect> draw_radarbbox = anchor_generate(delphi_point_in[it->second].pxy);
            //for(int m = 0; m < draw_radarbbox.size(); m++)
            //{
                //cv::rectangle(cv_ptr->image, draw_radarbbox[m], CV_RGB(0,0,255), 4, cv::LINE_8, 0);
            //}
            msg_marks.push_back(pub_markerarray(id_count++, delphi_point_in[it->second], b_in[it->first].Class));
            //break;
            //}
        }
        msg_markerarray.markers = msg_marks;
        pub_mark.publish(msg_markerarray);
        //msg_marks.clear();
        matchmap.clear();
        ioumap.clear();
    }
    //显示剩余yolo检测框
    for(int j = 0; j < b_in.size(); j++)
    {
        if(!draw_bbox_flag[j])
        {
            cv::Rect rect_in(b_in[j].xmin, b_in[j].ymin, 
                            b_in[j].xmax-b_in[j].xmin, b_in[j].ymax-b_in[j].ymin);
            cv::putText(cv_ptr->image, b_in[j].Class.c_str(), cv::Point(b_in[j].xmin, b_in[j].ymin+40), 
                        cv::FONT_HERSHEY_TRIPLEX, 1, CV_RGB(255,255,255), 2); //cv::Scalar(b, g, r)
            cv::rectangle(cv_ptr->image, rect_in, CV_RGB(255,255,255), 4, cv::LINE_8, 0);
            //ROS_INFO("class:%s", b_in[j].Class.c_str());
        }
    }
    //显示剩余雷达点
    for(int j = 0; j < delphi_point_in.size(); j++)
    {
        if(!draw_radar_flag[j])
        {
            cv::circle(cv_ptr->image, cv::Point(delphi_point_in[j].pxy.x, 
                        delphi_point_in[j].pxy.y), 8, CV_RGB(255,255,255), -1);
            char str[16];
            sprintf(str, "%.2f", delphi_point_in[j].prange);
            cv::putText(cv_ptr->image, str, cv::Point(delphi_point_in[j].pxy.x, delphi_point_in[j].pxy.y), 
                        cv::FONT_HERSHEY_TRIPLEX, 1, CV_RGB(255,255,255), 2); //cv::Scalar(b, g, r)
        }
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
                cv::Point(delphi_point_in[j].pxy.x, delphi_point_in[j].pxy.y), 8, CV_RGB(255,255,255), -1);
        }
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
}
