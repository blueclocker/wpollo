/*
 * @Author: your name
 * @Date: 2021-11-16 13:53:22
 * @LastEditTime: 2021-11-16 23:34:42
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/test/src/subdeque.cpp
 */

#include "test/subdeque.h"
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subdeque");
    ros::NodeHandle nh;


    /*TopicStack<std_msgs::String> test;
    test.init("topic", 3);
    std_msgs::String a;
    a.data = "a";
    std_msgs::String b;
    b.data = "bb";
    std_msgs::String c;
    c.data = "cccc";
    test.push_back(a);
    test.push_back(b);
    test.push_back(c);
    test.push_back(c);
    test.push_back(a);
    //test.pop_front();
    test.pop(3);
    std::cout << test.size() << std::endl;
    if(test.find(3) != nullptr) std::cout << *test.find(3);
    test.show(0);
    if(test.empty()) std::cout << "true" << std::endl;*/

    boost::shared_ptr<TopicStack<sensor_msgs::PointCloud2>> pc(new TopicStack<sensor_msgs::PointCloud2>);
    //TopicStack<sensor_msgs::PointCloud2> *pc = new TopicStack<sensor_msgs::PointCloud2>;

    pc->init("/velodyne_points", 5);

    //pc->subscribing(nh);
    boost::thread th(boost::bind(&TopicStack<sensor_msgs::PointCloud2>::subscribing, pc, nh));
    boost::bind(&TopicStack<sensor_msgs::PointCloud2>::subscribing, _1, _2)(pc, nh);
    th.join();

    //ros::spin();
    
    //delete pc;
    return 0;
}
