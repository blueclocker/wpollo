#include <iostream>
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
ros::Publisher pub;
bool flag = false;

void callback(const sensor_msgs::Image::ConstPtr &msg, 
              const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg_bboxes)
{
    //ros msg -> cv::mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    darknet_ros_msgs::BoundingBox bbox;
    for(int i = 0; i < (msg_bboxes->bounding_boxes).size(); i++)
    {
      bbox = msg_bboxes->bounding_boxes[i];
      cv::Rect rect(bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin);
      cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 0, 255), 4, cv::LINE_8, 0);
      cv::putText(cv_ptr->image, bbox.Class.c_str(), cv::Point(bbox.xmin, bbox.ymin+40), 
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
    //ros msg -> cv::mat
    if(flag == false)
    {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("camera only!");
    }
    flag = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh, nh_("~");
    pub = nh_.advertise<sensor_msgs::Image>("opencv_image", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_camera(
      nh, "/galaxy_camera/image_raw", 10, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bboxes(
      nh, "/darknet_ros/bounding_boxes", 10, ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        darknet_ros_msgs::BoundingBoxes>syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_camera, sub_bboxes);
    //ros::Rate r(0.03);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Subscriber sub = nh.subscribe("/galaxy_camera/image_raw", 10, cameracallback);
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
    //r.sleep();
}

