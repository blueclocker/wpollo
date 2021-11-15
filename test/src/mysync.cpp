/*
 * @Author: your name
 * @Date: 2021-11-05 20:23:11
 * @LastEditTime: 2021-11-15 22:55:10
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/test/src/mysync.cpp
 */
#include "test/mysync.h"

Mysync::Mysync(ros::NodeHandle &nh): n(nh)
{
    pub = nh.advertise<sensor_msgs::Image>("opencv_image", 100);
    bbox_deque = new std::deque<darknet_ros_msgs::BoundingBoxes>;
    camera_deque = new std::deque<sensor_msgs::Image>;
    boost::thread th1(&Mysync::threadbbox, this);
    //ROS_INFO("thread 1");
    boost::thread th2(&Mysync::threadimg, this);
    //ROS_INFO("thread 2");
    boost::thread th3(&Mysync::threadfusion, this);
    //ROS_INFO("thread 3");
    th1.join();
    th2.join();
    //th3.join();
    th3.detach();
    //threadbbox();
    //threadimg();
    //threadfusion();
    ros::spin();
}

Mysync::~Mysync()
{
    delete bbox_deque;
    delete camera_deque;
}

void Mysync::threadbbox()
{
    //boost::mutex::scoped_lock lock(bbox_mutex);
    sub_bboxes = n.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, &Mysync::bboxcallback, this);
    //boost::this_thread::sleep(boost::posix_time::seconds(1.0));
}

void Mysync::threadimg()
{
    //boost::mutex::scoped_lock lock(img_mutex);
    sub_camera = n.subscribe<sensor_msgs::Image>("/galaxy_camera/image_raw", 10, &Mysync::cameracallback, this);
    //boost::this_thread::sleep(boost::posix_time::seconds(1.0));
}

void Mysync::threadfusion()
{
    //boost::mutex::scoped_lock lock(img_mutex);
    ROS_INFO("in th3");
    int num_cam = 0, num_bbox = 0;
    while(n.ok())
    {
        boost::this_thread::sleep(boost::posix_time::millisec(50));
        if(!camera_deque->empty() && !bbox_deque->empty()) 
        {
            boost::mutex::scoped_lock lockimg(img_mutex);
            boost::mutex::scoped_lock lockbbox(bbox_mutex);
            if(find(num_cam, num_bbox))
            {
                callback(num_cam, num_bbox);
            }else{
                callback();
            }
            
        }else{
            
        }
    }
    ROS_INFO("leave th3");
}

bool Mysync::find(int &find_cam, int &find_bbox)
{
    ros::Time atnow = ros::Time::now();
    ros::Duration differ = ros::Duration(0.8);
    ros::Duration minvalue = ros::Duration(5);
    if((atnow - camera_deque->back().header.stamp) > differ || (atnow - bbox_deque->back().header.stamp) > differ)
    {
        return false;
    }else{
        for(int i = 0; i < camera_deque->size(); i++)
        {
            for(int j = 0; j < bbox_deque->size(); j++)
            {
                ros::Duration temp = camera_deque->at(i).header.stamp - bbox_deque->at(j).header.stamp;
                if(std::fabs(temp.toSec()) < minvalue.toSec())
                {
                    minvalue = temp;
                    find_cam = i;
                    find_bbox = j;
                }
            }
        }
        //find_cam = camera_deque->size() - 1;
        //find_bbox = bbox_deque->size() - 1;
        if(minvalue != ros::Duration(5)) return true;
    }
}

void Mysync::bboxcallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msgs)
{
    boost::this_thread::sleep(boost::posix_time::millisec(20));
    boost::mutex::scoped_lock lock(bbox_mutex);
    bbox_deque->push_back(*msgs);
    //ROS_INFO("bbox number = %d", (int)bbox_deque->size());
    if(bbox_deque->size() > 3) bbox_deque->pop_front();
    //std::cout << bbox_deque.front().boxes[0].Class << std::endl;
}

void Mysync::cameracallback(const sensor_msgs::Image::ConstPtr &msg)
{
    boost::this_thread::sleep(boost::posix_time::millisec(20));
    boost::mutex::scoped_lock lock(img_mutex);
    camera_deque->push_back(*msg);
    //ROS_INFO("img number = %d", (int)camera_deque->size());
    if(camera_deque->size() > 2) camera_deque->pop_front();
    //std::cout << camera_deque.front().height << std::endl;
}

void Mysync::callback(const int cam_in, const int bbox_in)
{
    //ros msg -> cv::mat
    //boost::mutex::scoped_lock lockimg(img_mutex);
    //boost::mutex::scoped_lock lockbbox(bbox_mutex);
    cv_bridge::CvImagePtr cv_ptr;
    //ROS_INFO("bbox number = %d", (int)bbox_queue->size());
    cv_ptr = cv_bridge::toCvCopy(camera_deque->at(cam_in), sensor_msgs::image_encodings::BGR8);
    darknet_ros_msgs::BoundingBox bbox;
    //std::cout << bbox_queue->size() << std::endl;
    for (int i = 0; i < (bbox_deque->at(bbox_in).bounding_boxes).size(); i++)
    {
        bbox = bbox_deque->at(bbox_in).bounding_boxes[i];
        cv::Rect rect(bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin);
        cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 0, 255), 4, cv::LINE_8, 0);
        //char str[16];
        //sprintf(str, "%s", bbox.Class.c_str());
        //cv::putText(cv_ptr->image, str, cv::Point(bbox.xmin, bbox.ymin + 40),
        //            cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(0, 0, 255), 2);
        //ROS_INFO("class:%s", bbox.Class.c_str());
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("fusion!");
}

void Mysync::callback()
{
    //boost::mutex::scoped_lock lock(img_mutex);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(camera_deque->back(), sensor_msgs::image_encodings::BGR8);
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub.publish(*msg_);
    ROS_INFO("only camera!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mysync");
    ros::NodeHandle nh_("~");
    Mysync my_sync(nh_);
    return 0;
}
