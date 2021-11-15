#include <iostream>
#include <thread>
#include <mutex>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
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
#include "test/Hungarian.h"

//sugar
/*
ros::Publisher pub;
ros::Subscriber sub_radar;
cv_bridge::CvImagePtr cv_ptr;
bool flag = false;

void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg_bboxes)
{
    darknet_ros_msgs::BoundingBox bbox;
    for(int i = 0; i < (msg_bboxes->bounding_boxes).size(); i++)
    {
      bbox = msg_bboxes->bounding_boxes[i];
      cv::Rect rect(bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin);
      cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 0, 255), 4, cv::LINE_8, 0);
      cv::putText(cv_ptr->image, bbox.Class.c_str(), cv::Point(bbox.xmin, bbox.ymin-15), 
                        cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(0, 0, 255), 2);
      ROS_INFO("class:%s", bbox.Class.c_str());
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("camera and radar!");
    flag = true;
}

void cameracallback(const sensor_msgs::Image::ConstPtr &msg)
{
    ros::NodeHandle nh_;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sub_radar = nh_.subscribe("/darknet_ros/bounding_boxes", 10, callback);
    if(flag == false)
    {
      sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
      pub.publish(*msg_);
      ROS_INFO("camera only!");
    }
    flag = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::Image>("opencv_image", 100);
    ros::Subscriber sub = nh.subscribe("/galaxy_camera/image_raw", 10, cameracallback);
    ros::spin();
}

*/
//wp
struct pixel_position
{
    //x
    int x;
    //y
    int y;
};
ros::Publisher pub;
bool flag = false;
bool first_frame = true;
int max_number = 0;
std::vector<pixel_position> last_frame;
std::vector<int> obj_ID;
boost::mutex io_mutex; 

double distance(const pixel_position pos_a, const pixel_position pos_b)
{
    return std::sqrt((pos_a.x - pos_b.x) * (pos_a.x - pos_b.x) +
                     (pos_a.y - pos_b.y) * (pos_a.y - pos_b.y));
}

void match(const std::vector<darknet_ros_msgs::BoundingBox> *bbox_hung)
{
    ROS_INFO("hung matching ...");
    //第一有效帧处理
    if(first_frame)
    {
        for(int i = 0; i < bbox_hung->size(); i++)
        {
            obj_ID.push_back(i);
            max_number = i;
        }
        ROS_INFO("first frame!!!");
        first_frame = false;
        return;
    }
    //ROS_INFO("bbox_hung size = %d", bbox_hung.size());
    std::vector<pixel_position> bbox_hung_in;
    for (int i = 0; i < bbox_hung->size(); i++)
    {
        pixel_position temp;
        temp.x = (int)((bbox_hung->at(i).xmin + bbox_hung->at(i).xmax) / 2);
        temp.y = (int)((bbox_hung->at(i).ymin + bbox_hung->at(i).ymax) / 2);
        bbox_hung_in.push_back(temp);
    }
    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;
    std::vector<std::vector<double> > costMatrix;
    double cost;
    if(!last_frame.empty())
    {
        for(int i = 0; i < bbox_hung_in.size(); i++)
        {
            std::vector<double> costmatrix_line;
            for(int j = 0; j < last_frame.size(); j++)
            {
                costmatrix_line.push_back(distance(bbox_hung_in[i], last_frame[j]));
            }
            costMatrix.push_back(costmatrix_line);
        }
        cost = HungAlgo.Solve(costMatrix, assignment);
        std::vector<int> temp_ID = obj_ID;
        obj_ID.resize(assignment.size());
        for (int x = 0; x < costMatrix.size(); x++)
        {
            //ROS_INFO("assignment[%d] = %d", x, assignment[x]);
            if(assignment[x] != -1)//匹配成功
            {
                if(costMatrix[x][assignment[x]] <= 100)//距离约束
                {
                    obj_ID[x] = temp_ID[assignment[x]];
                    //ROS_INFO("true!");
                }else{//超出匹配约束
                    cost -= costMatrix[x][assignment[x]];
                    obj_ID[x] = max_number;
                    ++max_number;
                    //ROS_INFO("false!");
                }
            }else{//匹配失败
                obj_ID[x] = max_number;
                ++max_number;
                //ROS_INFO("false");
            }
        }
        ROS_INFO("cost = %f", cost);
        last_frame = bbox_hung_in;
    }else{
        obj_ID.resize(bbox_hung_in.size());
        for(int i = 0; i < bbox_hung_in.size(); i++)
        {
            obj_ID[i] = max_number++;
        }
        last_frame = bbox_hung_in;
    }
    //ROS_INFO("max_number = %d", max_number);
}

void callback(const sensor_msgs::Image::ConstPtr &msg,
              const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg_bboxes)
{
    boost::mutex::scoped_lock lock(io_mutex);
    //ros msg -> cv::mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    darknet_ros_msgs::BoundingBox bbox;
    match(&msg_bboxes->bounding_boxes);
    for (int i = 0; i < (msg_bboxes->bounding_boxes).size(); i++)
    {
        bbox = msg_bboxes->bounding_boxes[i];
        cv::Rect rect(bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin);
        cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 0, 255), 4, cv::LINE_8, 0);
        char str[16];
        sprintf(str, "%d %s", obj_ID[i], bbox.Class.c_str());
        cv::putText(cv_ptr->image, str, cv::Point(bbox.xmin, bbox.ymin + 40),
                    cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(0, 0, 255), 2);
        //ROS_INFO("class:%s", bbox.Class.c_str());
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("camera and radar!");
    flag = true;
}

void cameracallback(const sensor_msgs::Image::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(io_mutex);
    //ros msg -> cv::mat
    if (flag == false)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
        pub.publish(*msg_);
        last_frame.clear();
        ROS_INFO("camera only!");
    }
    //ROS_INFO("in cameracallback!");
    flag = false;
    //last_frame.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh, nh_("~");
    pub = nh_.advertise<sensor_msgs::Image>("opencv_image", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_camera(
        nh, "/galaxy_camera/image_raw", 10, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bboxes(
        nh, "/darknet_ros/bounding_boxes", 10, ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            darknet_ros_msgs::BoundingBoxes>
        syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_camera, sub_bboxes);
    //ros::Rate r(0.03);
    sync.registerCallback(boost::bind(callback, _1, _2));
    //ros::Subscriber sub = nh.subscribe("/galaxy_camera/image_raw", 10, cameracallback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/galaxy_camera/image_raw", 10, boost::bind(cameracallback, _1));
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
    //r.sleep();
}
