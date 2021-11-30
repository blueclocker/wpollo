/*
 * @Author: your name
 * @Date: 2021-11-18 16:22:21
 * @LastEditTime: 2021-11-18 17:29:37
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/test/include/test/myfusion.h
 */
#ifndef MYFUSION_H_
#define MYFUSION_H_

#include <test/subdeque.h>
#include <vector>

struct TopicList
{
    std::string TopicName;
    std::string TopicType;
    int Queueline;
};

template <typename T, typename... Args>
class SubscribeStack
{
private:
    int number;
    ros::NodeHandle n;
    std::vector<TopicList> topics;
    TopicStack<std::string> a;
    std::vector<ros::Subscriber> subscribers;
public:
    void start();
    SubscribeStack();
    ~SubscribeStack();
};

template <typename T, typename... Args>
SubscribeStack<T, Args...>::SubscribeStack()
{
}

template <typename T, typename... Args>
SubscribeStack<T, Args...>::~SubscribeStack()
{
}

template <typename T, typename... Args>
void SubscribeStack<T, Args...>::start()
{
    for(int i = 0; i < number; ++i)
    {
        TopicStack<T> pc;
    }
}


#endif