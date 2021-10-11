/*
 * @Author: wpbit
 * @Date: 2021-09-08 19:22:26
 * @LastEditTime: 2021-10-11 09:25:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/camera_radar/include/camera_radar/camera_radar.h
 */

/*
 *                                                     __----~~~~~~~~~~~------___
 *                                    .  .   ~~//====......          __--~ ~~
 *                    -.            \_|//     |||\\  ~~~~~~::::... /~
 *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
 *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
 *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *           '         ~-|      /|    |-~\~~       __--~~
 *                       |-~~-_/ |    |   ~\_   _-~            /\
 *                            /  \     \__   \/~                \__
 *                        _--~ _/ | .-~~____--~-/                  ~~==.
 *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                  -_     ~\      ~~---l__i__i__i--~~_/
 *                                  _-~-__   ~)  \--______________--~~
 *                                //.-~~~-~_--~- |-------~~~~~~~~
 *                                       //.-~~~--\
 *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                               神兽保佑            永无BUG
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <eigen3/Eigen/Core>
//ROS
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <can_msgs/delphi_msg.h>
#include <can_msgs/delphi_msges.h>
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
//两传感器时间同步
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image,can_msgs::delphi_msges>two_syncPolicy;
//三传感器时间同步
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image,darknet_ros_msgs::BoundingBoxes,can_msgs::delphi_msges>three_syncPolicy;
//自定义单个雷达点像素坐标结构
struct pixel_position
{
        //x
        int x;
        //y
        int y;
};
//单个雷达点信息
typedef struct radar_info
{
        //像素坐标
        pixel_position pxy;
        //距离
        double prange;
        //反射率
        //double pdb;
        //速度
        double pspeed;
}radarinfo;
//单帧匹配结果
typedef struct detected_obj
{
        //相机数据
        sensor_msgs::Image::ConstPtr img;
        //YOLO检测框
        std::vector<darknet_ros_msgs::BoundingBox> bbox;
        //滤波后有效雷达点
        std::vector<radarinfo> del_point;
        //同matchmap
        std::unordered_map<int, int> map_result;
}det_obj;

class CameraRadarCore
{
    private:
        //相机宽、高
        int WIDTH, HEIGHT;
        //iou匹配阈值
        double IOU_KEY;
        //雷达框预定义大小
        double CAR_WIDTH, CAR_HEIGHT;
        //内参矩阵
        Eigen::Matrix<double, 3, 3> camera_matrix;
        //外参矩阵
        Eigen::Matrix<double, 4, 4> trans_matrix;
        //远距外参矩阵
        Eigen::Matrix<double, 4, 4> trans_matrix_long;
        //判断有无bbox
        bool bbox_flag = false;
        ros::NodeHandle nh;
        //融合结果发布器
        ros::Publisher pub;
        //匹配结果保存于map,前一个int是bbox索引，后一个int是雷达ID索引
        std::unordered_map<int, int> matchmap;
        //Header **暂时未用**
        std_msgs::Header ros_header;
        //订阅相机
        message_filters::Subscriber<sensor_msgs::Image> *sub_camera;
        //订阅yolo检测框
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> *sub_bboxes;
        //订阅雷达
        message_filters::Subscriber<can_msgs::delphi_msges> *sub_radar;
        //ROS时间同步
        message_filters::Synchronizer<two_syncPolicy> *two_sync;
        message_filters::Synchronizer<three_syncPolicy> *three_sync;
        //设置相机内参和外参
        bool set_param();
        //雷达无效点初步去除,由雷达原始数据转换三维空间点及速度(1帧)
        std::vector<can_msgs::delphi_msg> radar_filter(const std::vector<can_msgs::delphi_msg> delphi_in);
        //单个雷达坐标点投影
        pixel_position position_transform(const geometry_msgs::Point in_3d);
        //极坐标转geometry_msgs::Point
        geometry_msgs::Point polar_xy(const can_msgs::delphi_msg polar_xy_in);
        //空间同步函数，滤除雷达无效点后投影到像素坐标系
        std::vector<radarinfo> space_ok(const std::vector<can_msgs::delphi_msg> space_in_);
        //IOU计算
        double IOU(const cv::Rect &r1, const cv::Rect &r2);
        //图像坐标系两点之间欧式距离计算
        double distance(const pixel_position pos_a, const pixel_position pos_b);
        //最近邻匹配
        void knn_match(const std::vector<darknet_ros_msgs::BoundingBox> bbox_knn, const std::vector<radarinfo> radar_knn);
        //iou匹配
        void iou_match(const std::vector<darknet_ros_msgs::BoundingBox> bbox_iou, const std::vector<radarinfo> radar_iou);
        //最近邻匹配
        void hung_match(const std::vector<darknet_ros_msgs::BoundingBox> bbox_hung, const std::vector<radarinfo> radar_hung);
        //有YOLO检测结果回调函数
        void three_Callback(const sensor_msgs::Image::ConstPtr &three_camera, 
                      const darknet_ros_msgs::BoundingBoxes::ConstPtr &three_bboxes,
                      const can_msgs::delphi_msges::ConstPtr &three_radar);
        //无YOLO检测结果回调函数
        void two_Callback(const sensor_msgs::Image::ConstPtr &two_img,
                      const can_msgs::delphi_msges::ConstPtr &two_radar);
        //结果显示函数
        void draw_picture(const sensor_msgs::Image::ConstPtr &msg_in, 
                const std::vector<darknet_ros_msgs::BoundingBox> &b_in,
                const std::vector<radarinfo> &delphi_point_in);
        //函数重载
        void draw_picture(const sensor_msgs::Image::ConstPtr &msg_in, 
                const std::vector<radarinfo> &delphi_point_in);

    public:
        CameraRadarCore(ros::NodeHandle &nh);
        ~CameraRadarCore();
};
