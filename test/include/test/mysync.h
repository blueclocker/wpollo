/*
 * @Author: your name
 * @Date: 2021-11-05 20:31:00
 * @LastEditTime: 2021-11-14 16:20:52
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/test/include/test/mysync.h
 */
#include <iostream>
#include <queue>
#include <deque>
#include <vector>
#include <pthread.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>  
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
//时间同步
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

        
struct pixel_position
{
    //x
    int x;
    int y;
};

class Mysync
{
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    boost::mutex img_mutex;
    boost::mutex bbox_mutex;
    boost::mutex fusion_mutex;
    ros::Subscriber sub_camera;
    ros::CallbackQueue camera_queue;
    std::deque<sensor_msgs::Image> *camera_deque;
    ros::Subscriber sub_bboxes;
    std::deque<darknet_ros_msgs::BoundingBoxes> *bbox_deque;
    void callback();
    void callback(const int cam_in, const int bbox_in);
    //寻找缓存队列中时间戳最近的相机数据与检测框的下标
    bool find(int &find_cam, int &find_bbox);
    void cameracallback(const sensor_msgs::Image::ConstPtr &msg);
    void bboxcallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msgs);
    void threadimg();
    void threadbbox();
    void threadfusion();
public:
    Mysync(ros::NodeHandle &nh);
    ~Mysync();
};

